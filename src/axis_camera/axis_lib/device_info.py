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


def _auth_candidates(enable_auth, username, password):
    candidates = [None]

    if username and password:
        if enable_auth:
            candidates.append((username, password))
        else:
            # When enable_auth is false some cameras still require auth for CGI endpoints.
            candidates.append((username, password))

    if password and username != 'root':
        fallback = ('root', password)
        if fallback not in candidates:
            candidates.append(fallback)

    return candidates


def _open_request(request, url, timeout, credentials):
    if credentials is None:
        return urlopen(request, timeout=timeout)

    auth_username, auth_password = credentials
    password_mgr = HTTPPasswordMgrWithDefaultRealm()
    password_mgr.add_password(None, url, auth_username, auth_password)
    opener = build_opener(
        HTTPDigestAuthHandler(password_mgr),
        HTTPBasicAuthHandler(password_mgr)
    )
    return opener.open(request, timeout=timeout)


def http_get(hostname, path, timeout=5, enable_auth=False, username='root', password='', logger=None):
    url = 'http://%s%s' % (hostname, path)
    request = Request(url)

    for credentials in _auth_candidates(enable_auth, username, password):
        try:
            response = _open_request(request, url, timeout, credentials)
            body = response.read()
            try:
                return body.decode('utf-8')
            except Exception:
                return body.decode('latin1', 'ignore')
        except HTTPError as exc:
            if exc.code == 401:
                continue
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

    if logger is not None:
        logger('http_get: %s returned status 401' % path)
    return None


def http_post_json(hostname, path, payload, timeout=5, enable_auth=False, username='root', password='', logger=None):
    url = 'http://%s%s' % (hostname, path)
    body = json.dumps(payload)
    if not isinstance(body, bytes):
        body = body.encode('utf-8')

    request = Request(url, data=body, headers={'Content-Type': 'application/json'})

    for credentials in _auth_candidates(enable_auth, username, password):
        try:
            response = _open_request(request, url, timeout, credentials)
            raw_body = response.read()
            try:
                return raw_body.decode('utf-8')
            except Exception:
                return raw_body.decode('latin1', 'ignore')
        except HTTPError as exc:
            if exc.code == 401:
                continue
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

    if logger is not None:
        logger('http_post_json: %s returned status 401' % path)
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
    def _parse_basic_info(response):
        try:
            payload = json.loads(response)
        except Exception as exc:
            if logger is not None:
                logger('get_basic_device_info: invalid json response: %s' % exc)
            return None

        if payload.get('error'):
            return None

        property_list = payload.get('data', {}).get('propertyList', {})
        if not property_list:
            return None

        model = str(property_list.get('ProdNbr', 'unknown'))
        serial = str(property_list.get('SerialNumber', 'unknown'))
        firmware = str(property_list.get('Version', 'unknown'))

        if model == 'unknown' and serial == 'unknown' and firmware == 'unknown':
            return None

        return {
            'model': model,
            'serial': serial,
            'firmware': firmware
        }

    # Try modern and legacy API versions.
    for api_version in ['1.3', '1.0']:
        response = http_post_json(
            hostname,
            '/axis-cgi/basicdeviceinfo.cgi',
            payload={'apiVersion': api_version, 'method': 'getAllProperties'},
            timeout=timeout,
            enable_auth=enable_auth,
            username=username,
            password=password,
            logger=logger
        )
        if response is None:
            continue

        info = _parse_basic_info(response)
        if info is not None:
            return info

    # Fallback for cameras/firmware exposing GET semantics.
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

    return _parse_basic_info(response)


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
