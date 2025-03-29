from migen import passive


@passive
def timeout(timeout):
    for _ in range(timeout):
        yield
    raise Exception(f"timeout after {timeout} cycles")


def assert_eq_before(signal, value, timeout):
    for _ in range(timeout):
        if (yield signal) == value:
            return
        yield
    raise Exception(f"timeout waiting for {signal}=={value} after {timeout} cycles")


def assert_neq_before(signal, value, timeout):
    for _ in range(timeout):
        if (yield signal) != value:
            return
        yield
    raise Exception(f"timeout waiting for {signal}!={value} after {timeout} cycles")


def wait_for(signal, value):
    while True:
        if (yield signal) == value:
            return
        yield
