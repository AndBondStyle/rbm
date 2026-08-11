Import("env")

def skip_from_build(node):
    file_path = node.get_path()

    if "ESP32HWEncoder.cpp" in file_path:
        print(f"--> Excluding library file from build: {file_path}")
        return None

    return node

env.AddBuildMiddleware(skip_from_build, "*")
