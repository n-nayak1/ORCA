import os

Import("env")

# Include embedded toolchain include paths in compile_commands.json.
# This is important for clangd resolving Arduino/framework/toolchain headers.
env.Replace(COMPILATIONDB_INCLUDE_TOOLCHAIN=True)

# Put compile_commands.json at the project root so clangd finds it automatically.
env.Replace(COMPILATIONDB_PATH=os.path.join("$PROJECT_DIR", "compile_commands.json"))
