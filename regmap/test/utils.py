from migen import passive


@passive
def timeout(timeout):
    """ wait for `timeout` cycles, then raise an exception
    Useful as a global infinite loop protection
    """
    for _ in range(timeout):
        yield
    raise Exception(f"timeout after {timeout} cycles")


def assert_eq_before(signal, value, timeout):
    """Return when `signal`==`value`
    If the assertion wasn't True before `timeout` cycles, raise an exception
    """
    for _ in range(timeout):
        if (yield signal) == value:
            return
        yield
    raise Exception(f"timeout waiting for {signal}=={value} after {timeout} cycles")


def assert_neq_before(signal, value, timeout):
    """Return when `signal`!=`value`
    If the assertion wasn't True before `timeout` cycles, raise an exception
    """
    for _ in range(timeout):
        if (yield signal) != value:
            return
        yield
    raise Exception(f"timeout waiting for {signal}!={value} after {timeout} cycles")


def wait_for(signal, value):
    """Wait indefinitely for for `signal`!=`value`
    """
    while True:
        if (yield signal) == value:
            return
        yield


def ep_push(ep, data, zero=True, timeout=None):
    """Push data into a stream Endpoint
    If `zero` is True, null all the fields not present in `data`
    """
    if zero:
        for sig, _ in ep.iter_flat():
            if sig == ep.ready:
                continue
            yield sig.eq(0)

    for k, v in data.items():
        yield getattr(ep, k).eq(v)

    if timeout is None:
        timeout = -1

    yield ep.valid.eq(1)
    yield
    while (yield ep.ready) == 0:
        if timeout == 0:
            raise Exception("Timeout trying to push {data} into {ep}")
        if timeout is not None:
            timeout -= 1
        yield
    yield ep.valid.eq(0)


def ep_pop(ep, expected_data={}, timeout=None):
    """Pop data from a stream Endpoint
    All (key, value) pairs from `expected_data` are validated equal with the data being pop.
    if `expected_data` is an empty dict, no data is verified
    """
    if timeout is None:
        timeout = -1

    yield ep.ready.eq(1)
    yield
    while (yield ep.valid) == 0:
        if timeout == 0:
            raise Exception("Timeout trying to pop from {ep}")
        if timeout is not None:
            timeout -= 1
        yield
    for k, v in expected_data.items():
        val = (yield getattr(ep, k))
        if (val != v):
            raise ValueError("Expected {k}={v} but readback value = {val}")
    yield ep.ready.eq(0)
