## Plan: Control de detectores AXIS

**User Requirements**

QUiero añadir dos funcionalidades nuevas al componente. Por un lado quiero poder activar y desactivar la detección de personas y vehículos. En este caso, como la detección se lleva a cabo en el dispositivo axis, lo único que quiero hacer es deshabilitar la publicación del detector. Para ello habilitaremos un servicio del tipo object_detection_msgs/ManageDetector, llamado ~activate_detector. Los valores que admite son: person_detector y vehicle_detector.
Por otro lado quiero un servicio que liste los detectores disponibles (person_detector, vehicle_detector) del tipo robotnik_msgs/GetStringLis, llamado ~get_detectors_name_list.
Por último, quiero que publique un topic de tipo object_detection_msgs/DetectorsState con el estado de los detectores disponibles (person_detector, vehicle_detector), llamado ~detectors_states.

**Description**

Añadir control local y global por tipo de detector al nodo AXIS sin modificar la detección ejecutada en la cámara. `person_detector` filtrará detecciones normalizadas como `human`; `vehicle_detector` filtrará `vehicle`. Los servicios y el estado serán globales para todos los canales, mientras los tópicos `~detectors/<channel>/status` seguirán publicándose a frecuencia fija, vacíos cuando los tipos recibidos estén desactivados.

**Steps**
1. Añadir `object_detection_msgs` como dependencia de compilación/exportación/ejecución en `package.xml`, incluirla en `find_package(catkin REQUIRED COMPONENTS ...)` y `catkin_package(CATKIN_DEPENDS ...)` de `CMakeLists.txt`. No crear mensajes ni servicios nuevos: reutilizar `object_detection_msgs/ManageDetector`, `object_detection_msgs/DetectorState`, `object_detection_msgs/DetectorsState` y `robotnik_msgs/GetStringList`.
2. Extender la configuración de `AxisDetectionNode` con parámetros privados `~person_detector_enabled` y `~vehicle_detector_enabled`, ambos `true` por defecto para conservar el comportamiento actual. Añadirlos a `axis_f2105re_detection_config.yaml` y al diccionario `arg_defaults`.
3. Mantener en `AxisDetectionNode` un estado ordenado y global con los dos nombres públicos exactos: `person_detector` y `vehicle_detector`. Centralizar el mapeo de clases en una constante (`human` -> `person_detector`, `vehicle` -> `vehicle_detector`) para evitar condicionales duplicados.
4. Registrar en `rosSetup()` el servicio privado `~activate_detector` de tipo `object_detection_msgs/ManageDetector`. El callback debe aceptar `person_detector`, `vehicle_detector` y el alias `all`; cambiar el estado de forma idempotente, responder `success=True` para nombres válidos y `success=False` con los nombres disponibles para cualquier otro valor. El cambio sólo afecta al filtrado/publicación ROS, no inicia ni detiene detectores en la cámara ni el WebSocket.
5. Registrar el servicio privado `~get_detectors_name_list` de tipo `robotnik_msgs/GetStringList`. Si `request.data` está vacío, devolver los estados de `person_detector` y `vehicle_detector`; si no está vacío, validar que corresponda exactamente a uno de esos nombres y devolver únicamente el estado del detector solicitado. Para un nombre no válido, responder con `ReturnMessage(success=False, message=...)`; para consultas válidas, devolver el nombre y estado actual mediante la respuesta del servicio, manteniendo el orden estable cuando se soliciten todos.
6. Crear en `rosSetup()` el publisher privado `~detectors_states` de tipo `object_detection_msgs/DetectorsState`. Construir siempre dos `DetectorState` con los nombres anteriores y sus flags actuales. Publicar este mensaje en cada ciclo de `publishStatus()`, usando la misma frecuencia `~rate`; opcionalmente publicar también inmediatamente tras una llamada válida a `~activate_detector` para reducir latencia de observación.
7. Aplicar el filtrado en `publishChannelDetections()` justo antes de convertir cada detección a `AxisMetadataDetection`: omitir `human` si `person_detector` está inactivo y `vehicle` si `vehicle_detector` está inactivo. Hacer una copia consistente del estado bajo lock para evitar carreras entre callbacks de servicios y el bucle de publicación.
8. Conservar la semántica de frecuencia fija existente: todos los canales configurados continúan publicando un `AxisMetadataDetectionArray` cada ciclo. Si no hubo detecciones o todas fueron filtradas por detectores inactivos, publicar el array vacío. Al desactivar un detector, descartar también cualquier detección pendiente de ese tipo mediante el filtrado en tiempo de publicación.
9. Actualizar `README.md` con los dos parámetros iniciales, el tópico `~detectors_states`, los servicios `~activate_detector` y `~get_detectors_name_list`, nombres admitidos, soporte de `all`, alcance global entre canales y aclaración de que sólo se deshabilita la publicación.
10. Añadir tests centrados en la lógica nueva. Para mantener tests simples, extraer helpers puros o probar el nodo con mocks mínimos de ROS: estado inicial configurable; activación/desactivación individual y `all`; rechazo de nombre desconocido; lista estable; construcción de `DetectorsState`; filtrado `human`/`vehicle`; y publicación vacía cuando el tipo está inactivo.

**Relevant files**
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/src/axis_camera/axis_detection_node.py` — estado de detectores, callbacks de servicios, publisher de estados y filtrado antes de publicar por canal.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/config/axis_f2105re_detection_config.yaml` — defaults iniciales configurables para ambos detectores.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/CMakeLists.txt` — dependencia catkin de `object_detection_msgs`.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/package.xml` — dependencias build/export/exec de `object_detection_msgs`.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/test/test_axis_detection.py` y/o un nuevo test específico del nodo — cobertura de estado, servicios y filtrado.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/sensors/axis_camera/README.md` — contrato ROS y comportamiento operativo.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/msgs/object_detection_msgs/srv/ManageDetector.srv` — contrato existente: request `name`, `active`; response `success`, `message`.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/msgs/object_detection_msgs/msg/DetectorState.msg` y `DetectorsState.msg` — contratos existentes para estado.
- `/home/robot/workspaces/inspection_ws/src/inspection_packages/msgs/robotnik_msgs/srv/GetStringList.srv` — contrato existente para listar nombres.

**Verification**
1. Ejecutar tests Python del paquete con `python3 -m unittest discover -s test` y confirmar casos individuales, `all`, nombre desconocido, filtrado y mensajes vacíos.
2. Ejecutar `python3 -m py_compile src/axis_camera/axis_detection_node.py src/axis_camera/axis_lib/axis_detection.py`.
3. Compilar con `catkin build axis_camera` para validar imports y dependencias generadas de `object_detection_msgs`/`robotnik_msgs`.
4. Ejecutar `catkin run_tests axis_camera` y comprobar cero fallos.
5. Prueba ROS manual: consultar `rosservice call /axis_detection/get_detectors_name_list`, verificar los dos nombres; desactivar `person_detector` con `ManageDetector`; confirmar que `~detectors_states` refleja `false`, que desaparecen detecciones `human` de todos los canales y que cada tópico por canal sigue publicando arrays vacíos a `rate` cuando no quedan detecciones.
6. Repetir con `vehicle_detector`, con `all`, con una llamada idempotente y con un nombre inválido; confirmar respuestas y que el WebSocket permanece conectado.

**Decisions**
- La activación es global para todos los canales del nodo, porque `ManageDetector` no contiene un campo de canal.
- Los estados iniciales son configurables mediante `person_detector_enabled` y `vehicle_detector_enabled`, con default `true`.
- `~activate_detector` acepta `person_detector`, `vehicle_detector` y `all`; otros nombres se rechazan.
- `person_detector` corresponde a `class_label == 'human'`; `vehicle_detector` corresponde a `class_label == 'vehicle'`.
- Desactivar significa suprimir publicaciones de ese tipo; la cámara AXIS y el WebSocket continúan operativos.
- `~detectors_states` se publica a la frecuencia existente `~rate` y representa disponibilidad/configuración global, no estado de conexión ni presencia de detecciones.
- No se modifican las interfaces existentes de `object_detection_msgs` ni `robotnik_msgs`.
