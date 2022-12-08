# AndorNetwork

Access Andor SDK functions remotely via Ethernet.

This is a thin wrapper around the [Andor SDK](https://andor.oxinst.com/products/software-development-kit/) providing Remote Procedure Call (RPC) through [ZeroC Ice](https://github.com/zeroc-ice/ice), written in C++11.

## Feature

- Cross platform. Can be deployed to any languages Ice supports (e.g. python, Matlab, Java, etc.).

- Return values directly instead of passing reference.

- Throw exceptions if a call failed.

- Maximally preserved the call signatures of Andor SDK functions to minimize migration effort (but got rid of all pointers).

- Complete separation between calls to Andor SDK and users' program makes debugging and development easier.

- Industrial standard RPC library ZeroC Ice provides superior performance and stability.

## Usage

### Run the server

Binaries for x64 Windows platform can be found at Release page.

Users need to copy `atmcd64d.dll` to the folder of the binaries before running the executable file. This file belongs to Andor SDK. Since it is proprietary, we can not include it in the release (but it can be found inside the installation folder of [Micro-Manager](https://micro-manager.org/).)

The excutable file `AndorNetwork.exe` can be run from `cmd` or a terminal.

```
ZeroC-ICE Wrapper of Andor SDK

Usage: AndorNetwork.exe <option(s)>
Options:
        -h,--help               Show this help message
        -b,--bind BIND  Specify the interface to bind to. Default: `*`, i.e. listen to all interface.
        -p,--port PORT  Specify the port to bind to. Default: `5566`.
```

### Python client example

See [python/client.py](python/client.py) (included in the release, readily usable). `pip install zeroc-ice` is reqired.

If you are building everything from scratch, you also need to generate the required interface files by:

- `cd python`, and

- `slice2py ../src/Andor.ice`

### Function reference

AndorNetwork supports nearly all Andor SDK functions defined in `atmcd32d.h`. However, not all of these function wrappers are tested (most of them are actually generated code).

Call signatures can be found in the Ice interface definition file `src/Andor.ice`. It's [syntax](https://doc.zeroc.com/ice/3.7/the-slice-language) is very close to C++ and is easily readable.

## Build

The build process is streamlined with the help of CMake.

1. Install [Microsoft C++ Build Tools](https://visualstudio.microsoft.com/visual-cpp-build-tools/), select _Desktop development with C++_. Make sure the following optional items are checked:
   
   1. MSVC,
   
   2. Windows 10 SDK,
   
   3. C++ CMake tools for Windows.

2. There is a chance that your `cmake` is not in `PATH`. Follow the instruction [here](https://stackoverflow.com/questions/61625880/vscode-cmake-windows-10-cmake-not-in-path) to add it to `PATH`. You can also install `cmake` through [choco](https://community.chocolatey.org/packages/cmake) and it will be automatically put into `PATH`. Make sure once you open a terminal and type `cmake`, it shows up.

3. Install [NuGet commandline tool](https://www.nuget.org/downloads) and put it into your `PATH`. I recommend using [choco](https://community.chocolatey.org/packages/NuGet.CommandLine) because it automatically adds it into `PATH`.

4. Clone this repo locally and `cd` into it. Create a folder called `build`.

5. **Copy `atmcd64d.dll` and `atmcd64m.lib` into `lib` folder.**

6. Run `cmake -Bbuild .` to generate all Makefiles.

7. `cd build`, then `cmake --build .`

8. If everything goes well, binaries will be inside `build/Debug`. But you need to copy all required DLLs to this folder before running the executable. It includes `atmcd64d.dll` and some DLLs from Ice (can be found inside `packages/zeroc.ice.v140.3.7.8/build/native/bin/x64/Debug/`).






