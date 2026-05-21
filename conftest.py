# conftest.py
# pytest setup - compiles the C source files into a shared library and loads it via ctypes
# every test depends on the drone_lib fixture defined here

import ctypes
import pathlib
import subprocess
import sys

import pytest

# add the parent directory to sys.path so we can import data_collection.py from there
_PARENT_DIR = pathlib.Path(__file__).resolve().parent.parent
if str(_PARENT_DIR) not in sys.path:
    sys.path.insert(0, str(_PARENT_DIR))

from drone_structs import (
    IMUData,
    EstimatorConfig,
    Estimator,
    PIDOutput,
    PIDState,
    PIDGain,
    MotorOutput,
)


# C source files live one level up from tests/
SOURCE_DIR = pathlib.Path(__file__).resolve().parent.parent


@pytest.fixture(scope="session")
def drone_lib():
    # scope=session means this runs once per pytest invocation (not per test)
    # compiling C is slow, we only want to do it once

    # hidden build directory so it doesn't clutter the source folder
    build_dir = SOURCE_DIR / ".pytest_build"
    build_dir.mkdir(exist_ok=True)
    lib_path = build_dir / "libdrone_controller.so"

    # gcc command - build a shared library from the three .c files
    compile_command = [
        "gcc",
        "-shared", "-fPIC",          # shared library flags
        "-O2", "-std=c99",           # standard compile flags
        "-Wall", "-Wextra",          # show all warnings
        "-I", str(SOURCE_DIR),       # include path for the .h files
        str(SOURCE_DIR / "estimator.c"),
        str(SOURCE_DIR / "pid.c"),
        str(SOURCE_DIR / "motor_mixer.c"),
        "-lm",                       # link math library
        "-o", str(lib_path),
    ]
    subprocess.run(compile_command, check=True, capture_output=True, text=True)

    # load the compiled .so file
    lib = ctypes.CDLL(str(lib_path))

    # tell ctypes the type signatures - otherwise it assumes int args and corrupts our floats

    lib.Estimator_Init.argtypes = []
    lib.Estimator_Init.restype = None

    lib.Estimator_SetConfig.argtypes = [ctypes.POINTER(EstimatorConfig)]
    lib.Estimator_SetConfig.restype = None

    lib.Estimator_Update.argtypes = [
        ctypes.POINTER(IMUData),
        ctypes.POINTER(Estimator),
    ]
    lib.Estimator_Update.restype = None

    # all six PID functions have the same signature - declare them in a loop
    pid_arg_types = [
        ctypes.POINTER(PIDGain),
        ctypes.POINTER(Estimator),
        ctypes.POINTER(PIDOutput),
        ctypes.POINTER(PIDState),
    ]
    for pid_function_name in ("roll_PID", "pitch_PID", "yaw_PID",
                              "accX_PID", "accY_PID", "accZ_PID"):
        function = getattr(lib, pid_function_name)
        function.argtypes = pid_arg_types
        function.restype = None

    lib.motor_mixer.argtypes = [
        ctypes.POINTER(MotorOutput),
        ctypes.POINTER(Estimator),
        ctypes.POINTER(PIDOutput),
    ]
    lib.motor_mixer.restype = None

    return lib