import pytest

from dynova import find_wasm


def pytest_sessionstart(session):
    # A missing wasm must fail the run, not skip every test and report success.
    find_wasm()
