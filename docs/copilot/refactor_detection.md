## Plan: Separar Detección Axis

Refactorizar `AxisPTZ` para que vuelva a centrarse en PTZ/foco/iris/imagen, moviendo la integración de metadata analytics de AXIS a una clase reutilizable y a un nodo ROS independiente. El nuevo nodo publicará `~detectors/status` bajo su propio namespace, sin mantener compatibilidad con el tópico relativo anterior del nodo PTZ.

**Steps**
1. Inventariar y aislar el bloque de detección actual en `axis_ptz_node.py`: imports `json`, `ssl`, `websocket`, mensajes `AxisMetadataDetection*`, atributos `detection_*`, métodos `_normalizeDetectionChannelFilter` a `_stopDetectionStream`, llamada a `_startDetectionStream()` y hook `rospy.on_shutdown(self._stopDetectionStream)`, defaults `detection_*` en `main()`.
2. Crear `src/axis_camera/axis_lib/axis_detection.py` con una clase simple, por ejemplo `AxisDetectionClient`, responsable de WebSocket VAPIX: construir URL/configure payload, normalizar `channel_filter`, reconectar, manejar 404 como no soportado, parsear observaciones y emitir resultados ya normalizados mediante callbacks. Mantener esta clase sin dependencias ROS para que sea testeable.
3. Diseñar la salida del cliente como datos simples: lista de detecciones con `track_id`, `class_label`, `score`, `left`, `top`, `right`, `bottom`. Mantener la normalización actual de clases (`human`/`vehicle`) y descartar clases desconocidas antes o durante la conversión en el nodo.
4. Crear `src/axis_camera/axis_detection_node.py` siguiendo el estilo de `axis_stream_node.py` y `axis_ptz_node.py`: `main()` lee parámetros, clase de nodo hace `rosSetup()`, crea publisher `~detectors/status`, instancia el cliente de detección, registra shutdown y ejecuta hasta `rospy.is_shutdown()`.
5. En el nuevo nodo, publicar `robotnik_msgs/AxisMetadataDetectionArray` en `~detectors/status` bajo el namespace propio del nodo. Replicar el comportamiento actual de publicar sólo detecciones válidas `human`/`vehicle`; decidir si el publisher se crea siempre en `rosSetup()` para visibilidad del nuevo nodo o se conserva creación perezosa tras configure ACK. Recomendación: crear publisher en `rosSetup()` y usar logs/diagnostics para estado, porque el nodo ya existe específicamente para detección.
6. Refactorizar `src/axis_camera/axis_ptz_node.py` eliminando la lógica de detección: imports no usados, atributos `detection_*`, todos los callbacks/métodos WebSocket, llamada de arranque/parada, parámetros `detection_*` del diccionario de defaults. Verificar que `AxisPTZ` conserva sólo PTZ, foco, iris, image settings, diagnostics y device info.
7. Añadir instalación del nuevo script en `CMakeLists.txt` con `catkin_install_python(PROGRAMS src/axis_camera/axis_detection_node.py ...)`, siguiendo el patrón existente para `axis_ptz_node.py` y `axis_stream_node.py`.
8. Actualizar dependencias runtime en `package.xml`: añadir la dependencia de sistema ROS correspondiente a `websocket-client` si existe en el entorno objetivo, o documentar explícitamente la instalación si no hay rosdep key fiable. Recomendación inicial: usar `python-websocket-client` para ROS/Python 2 si este workspace sigue en ROS 1 clásico.
9. Crear `launch/axis_detection.launch` con argumentos `node_name`, `ip_address`, `detection_enabled`, `detection_use_tls`, `detection_ws_source`, `detection_channel_filter` y nodo `axis_detection_node.py`. No modificar `axis_ptz.launch` para arrancar detección automáticamente salvo que el despliegue quiera incluir ambos; mantener los nodos separados.
10. Crear configuración opcional dedicada, por ejemplo `config/axis_detection_config.yaml`, sólo con parámetros de detección. Evitar mezclarlos en `*_ptz_config.yaml` para que la separación sea clara.
11. Actualizar `README.md`: mover la sección “Detection metadata” fuera del apartado de PTZ, documentar `axis_detection_node.py`, su launch, parámetros, tópico publicado bajo el namespace nuevo y el hecho de que `axis_ptz_node.py` ya no integra detección.
12. Añadir pruebas unitarias enfocadas para la clase nueva: normalización de `detection_channel_filter`, payload configure, extracción de observaciones, extracción/normalización de clase, filtrado de bounding boxes incompletos y manejo de clases desconocidas. Si el paquete no tiene infraestructura de test, crear la mínima compatible con catkin/rostest o dejar test puro Python ejecutable con `python -m unittest`.

**Relevant files**
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/src/axis_camera/axis_ptz_node.py` — retirar detección de `AxisPTZ`; referencias actuales: imports de metadata/WebSocket, atributos `detection_*`, métodos `_normalizeDetectionChannelFilter` a `_stopDetectionStream`, defaults `detection_*`.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/src/axis_camera/axis_lib/axis_detection.py` — nuevo cliente reusable de metadata AXIS, sin ROS.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/src/axis_camera/axis_detection_node.py` — nuevo nodo ROS independiente que convierte resultados del cliente a `AxisMetadataDetectionArray` y publica `~detectors/status`.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/CMakeLists.txt` — añadir `catkin_install_python` para el nuevo nodo.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/package.xml` — añadir/documentar dependencia runtime de `websocket-client`.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/launch/axis_detection.launch` — nuevo launch del nodo de detección.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/config/axis_detection_config.yaml` — configuración opcional dedicada a detección.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/README.md` — documentación del nuevo nodo y del cambio de responsabilidad.

**Verification**
1. Ejecutar tests unitarios del parser/normalizador nuevo, por ejemplo `python -m unittest discover -s test` si se crea carpeta `test`.
2. Ejecutar validación sintáctica Python sobre los ficheros tocados: `python -m py_compile src/axis_camera/axis_ptz_node.py src/axis_camera/axis_detection_node.py src/axis_camera/axis_lib/axis_detection.py`.
3. Ejecutar build catkin del paquete, por ejemplo desde el workspace: `catkin build axis_camera` o el comando equivalente usado por el repositorio.
4. Validar instalación de scripts: confirmar que `rosrun axis_camera axis_detection_node.py` encuentra el nuevo nodo después del build/source.
5. Prueba manual con cámara compatible: `roslaunch axis_camera axis_detection.launch ip_address:=<camera_ip>` y verificar `rostopic echo /<namespace_del_nuevo_nodo>/detectors/status`.
6. Prueba manual con cámara sin endpoint metadata: verificar log informativo de endpoint no soportado y que el nodo no cae.
7. Regresión PTZ: lanzar `axis_ptz.launch`, confirmar que servicios/tópicos PTZ siguen disponibles y que ya no intenta abrir WebSocket de detección.

**Decisions**
- El tópico de detección vivirá bajo el namespace del nuevo nodo; no se mantendrá bridge ni remap automático desde el namespace del nodo PTZ.
- La lógica reusable de WebSocket/parsing debe vivir en `axis_lib` y no depender de `rospy`.
- El objetivo es código claro y simple: una clase cliente para VAPIX metadata, un nodo ROS delgado para parámetros/publisher/lifecycle, y `AxisPTZ` sin responsabilidades de detección.
- Queda fuera de alcance cambiar mensajes `robotnik_msgs/AxisMetadataDetection*` o rediseñar el contrato de detecciones.

**Further Considerations**
1. Publisher perezoso vs publisher siempre visible: recomiendo publisher siempre visible en el nodo nuevo por claridad operacional; si se quiere conservar exactamente el comportamiento anterior, hacerlo perezoso tras configure ACK.
2. Dependencia `websocket-client`: verificar en la distro ROS objetivo si la rosdep key correcta es `python-websocket-client` o `python3-websocket-client` antes de cerrar package.xml.
3. Si existen consumidores actuales del tópico antiguo, deberán actualizar launch/remaps fuera de este paquete porque se decidió usar namespace nuevo.
