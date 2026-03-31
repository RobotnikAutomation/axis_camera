import json

try:
    import httplib
except Exception:
    import http.client as httplib

try:
    from urllib.request import (
        build_opener,
        HTTPPasswordMgrWithDefaultRealm,
        HTTPDigestAuthHandler,
        HTTPBasicAuthHandler,
        Request,
        urlopen,
    )
    from urllib.error import HTTPError, URLError
except Exception:
    from urllib2 import (
        build_opener,
        HTTPPasswordMgrWithDefaultRealm,
        HTTPDigestAuthHandler,
        HTTPBasicAuthHandler,
        Request,
        urlopen,
        HTTPError,
        URLError,
    )


def _encode_basic_auth(username, password):
    auth_string = '%s:%s' % (username, password)
    try:
        import base64
        encoded = base64.encodestring(auth_string)[:-1]
    except Exception:
        import base64
        encoded = base64.encodebytes(auth_string.encode())[:-1]

    if not isinstance(encoded, str):
        encoded = encoded.decode('ascii')

    return 'Basic %s' % encoded


def http_get(hostname, path, timeout=5, enable_auth=False, username='root', password='', logger=None):
    url = 'http://%s%s' % (hostname, path)
    request = Request(url)

    try:
        if enable_auth:
            password_mgr = HTTPPasswordMgrWithDefaultRealm()
            password_mgr.add_password(None, url, username, password)
            opener = build_opener(
                HTTPDigestAuthHandler(password_mgr),
                HTTPBasicAuthHandler(password_mgr)
            )
            response = opener.open(request, timeout=timeout)
        else:
            response = urlopen(request, timeout=timeout)

        body = response.read()
        try:
            return body.decode('utf-8')
        except Exception:
            return body.decode('latin1', 'ignore')
    except HTTPError as exc:
        if logger is not None:
            logger('http_get: %s returned status %s' % (path, exc.code))
        return None
    except URLError as exc:
        if logger is not None:
            logger('http_get: error getting %s: %s' % (path, exc))
        return None
    except Exception as exc:
        if logger is not None:
            logger('http_get: error getting %s: %s' % (path, exc))
        return None


def http_post_json(hostname, path, payload, timeout=5, enable_auth=False, username='root', password='', logger=None):
    url = 'http://%s%s' % (hostname, path)
    body = json.dumps(payload)
    if not isinstance(body, bytes):
        body = body.encode('utf-8')

    request = Request(url, data=body, headers={'Content-Type': 'application/json'})

    try:
        if enable_auth:
            password_mgr = HTTPPasswordMgrWithDefaultRealm()
            password_mgr.add_password(None, url, username, password)
            opener = build_opener(
                HTTPDigestAuthHandler(password_mgr),
                HTTPBasicAuthHandler(password_mgr)
            )
            response = opener.open(request, timeout=timeout)
        else:
            response = urlopen(request, timeout=timeout)

        raw_body = response.read()
        try:
            return raw_body.decode('utf-8')
        except Exception:
            return raw_body.decode('latin1', 'ignore')
    except HTTPError as exc:
        if logger is not None:
            logger('http_post_json: %s returned status %s' % (path, exc.code))
        return None
    except URLError as exc:
        if logger is not None:
            logger('http_post_json: error posting %s: %s' % (path, exc))
        return None
    except Exception as exc:
        if logger is not None:
            logger('http_post_json: error posting %s: %s' % (path, exc))
        return None


def get_param_cgi_values(hostname, path, timeout=5, enable_auth=False, username='root', password='', logger=None):
    response = http_get(hostname, path, timeout, enable_auth, username, password, logger)
    if response is None:
        return {}

    values = {}
    for raw_line in response.splitlines():
        line = raw_line.strip()
        if not line or '=' not in line:
            continue
        key, value = line.split('=', 1)
        values[key.strip()] = value.strip()

    return values


def get_first_value(values, keys, default='unknown'):
    for key in keys:
        value = values.get(key, '')
        if value:
            return value
    return default


def get_basic_device_info(hostname, timeout=5, enable_auth=False, username='root', password='', logger=None):
    response = http_post_json(
        hostname,
        '/axis-cgi/basicdeviceinfo.cgi',
        payload={'apiVersion': '1.3', 'method': 'getAllProperties'},
        timeout=timeout,
        enable_auth=enable_auth,
        username=username,
        password=password,
        logger=logger
    )

    # Fallback for older cameras/firmware that still provide GET semantics.
    if response is None:
        response = http_get(
            hostname,
            '/axis-cgi/basicdeviceinfo.cgi',
            timeout=timeout,
            enable_auth=enable_auth,
            username=username,
            password=password,
            logger=logger
        )

    if response is None:
        return None

    try:
        payload = json.loads(response)
        property_list = payload.get('data', {}).get('propertyList', {})
        return {
            'model': str(property_list.get('ProdNbr', 'unknown')),
            'serial': str(property_list.get('SerialNumber', 'unknown')),
            'firmware': str(property_list.get('Version', 'unknown'))
        }
    except Exception as exc:
        if logger is not None:
            logger('get_basic_device_info: invalid json response: %s' % exc)
        return None


def get_firmware_from_param_cgi(hostname, timeout=5, enable_auth=False, username='root', password='', logger=None):
    values = get_param_cgi_values(
        hostname,
        '/axis-cgi/admin/param.cgi?action=list&group=Properties.Firmware',
        timeout,
        enable_auth,
        username,
        password,
        logger
    )
    firmware = get_first_value(values, [
        'Properties.Firmware.Version',
        'Properties.Firmware.Build',
        'Properties.System.Firmware.Version',
        'root.Properties.Firmware.Version',
        'root.Properties.System.Firmware.Version'
    ], 'unknown')
    return firmware if firmware else 'unknown'


def get_brand_from_param_cgi(hostname, timeout=5, enable_auth=False, username='root', password='', logger=None):
    values = get_param_cgi_values(
        hostname,
        '/axis-cgi/admin/param.cgi?action=list&group=root.Brand',
        timeout,
        enable_auth,
        username,
        password,
        logger
    )

    brand = get_first_value(values, [
        'root.Brand.Brand',
        'Brand.Brand',
        'root.Brand.Manufacturer',
        'Properties.System.Brand.Brand'
    ], '').strip()
    prod_short = get_first_value(values, [
        'root.Brand.ProdShortName',
        'Brand.ProdShortName',
        'root.Brand.ProdNbr',
        'root.Brand.Product',
        'Properties.System.Brand.ProdShortName'
    ], '').strip()
    prod_full = get_first_value(values, [
        'root.Brand.ProdFullName',
        'Brand.ProdFullName',
        'Properties.System.Brand.ProdFullName'
    ], '').strip()
    serial = get_first_value(values, [
        'root.Brand.SerialNumber',
        'Brand.SerialNumber',
        'root.Properties.System.SerialNumber',
        'Properties.System.SerialNumber',
        'root.System.SerialNumber'
    ], 'unknown')

    model = prod_full or prod_short or brand or 'unknown'
    if brand and model != brand and not model.startswith(brand):
        model = '%s %s' % (brand, model)

    return model, serial if serial else 'unknown'


def get_device_info_with_fallback(hostname, timeout=5, enable_auth=False, username='root', password='', logger=None):
    model = 'unknown'
    serial = 'unknown'
    firmware = 'unknown'

    basic_info = get_basic_device_info(
        hostname,
        timeout=timeout,
        enable_auth=enable_auth,
        username=username,
        password=password,
        logger=logger
    )

    if basic_info is not None:
        model = basic_info.get('model', 'unknown')
        serial = basic_info.get('serial', 'unknown')
        firmware = basic_info.get('firmware', 'unknown')
    else:
        firmware = get_firmware_from_param_cgi(
            hostname,
            timeout=timeout,
            enable_auth=enable_auth,
            username=username,
            password=password,
            logger=logger
        )
        model, serial = get_brand_from_param_cgi(
            hostname,
            timeout=timeout,
            enable_auth=enable_auth,
            username=username,
            password=password,
            logger=logger
        )

    return {
        'model': model if model else 'unknown',
        'serial': serial if serial else 'unknown',
        'firmware': firmware if firmware else 'unknown'
    }
