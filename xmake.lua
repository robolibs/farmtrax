set_project("farmtrax")
set_version("1.1.1")
set_xmakever("2.7.0")

-- Set C++ standard
set_languages("c++20")

-- Add build options
add_rules("mode.debug", "mode.release")

-- Compiler warnings and flags (matching CMake)
add_cxxflags("-Wno-all", "-Wno-extra", "-Wno-pedantic", "-Wno-maybe-uninitialized", "-Wno-unused-variable", "-Wno-reorder")

-- Add global search paths for packages in ~/.local
local home = os.getenv("HOME")
if home then
    add_includedirs(path.join(home, ".local/include"))
    add_linkdirs(path.join(home, ".local/lib"))
end

-- Add devbox/nix paths for system packages
local cmake_prefix = os.getenv("CMAKE_PREFIX_PATH")
if cmake_prefix then
    add_includedirs(path.join(cmake_prefix, "include"))
    add_linkdirs(path.join(cmake_prefix, "lib"))
end

local pkg_config = os.getenv("PKG_CONFIG_PATH")
if pkg_config then
    -- Split PKG_CONFIG_PATH by ':' and process each path
    for _, pkgconfig_path in ipairs(pkg_config:split(':')) do
        if os.isdir(pkgconfig_path) then
            -- PKG_CONFIG_PATH typically points to .../lib/pkgconfig
            -- We want to get the prefix (two levels up) to find include and lib
            local lib_dir = path.directory(pkgconfig_path)  -- .../lib
            local prefix_dir = path.directory(lib_dir)      -- .../
            local include_dir = path.join(prefix_dir, "include")

            if os.isdir(lib_dir) then
                add_linkdirs(lib_dir)
            end
            if os.isdir(include_dir) then
                add_includedirs(include_dir)
            end
        end
    end
end

-- Options
option("examples")
    set_default(false)
    set_showmenu(true)
    set_description("Build examples")
option_end()

option("tests")
    set_default(false)
    set_showmenu(true)
    set_description("Enable tests")
option_end()

-- Define datapod package (from local path)
package("datapod")
    set_kind("library", {headeronly = true})
    set_sourcedir("/doc/ares/datapod")

    on_fetch(function (package)
        local result = {}
        result.includedirs = {"/doc/ares/datapod/include"}
        return result
    end)
package_end()

-- Define concord package (from local path - NEW version)
package("concord")
    set_kind("library", {headeronly = true})
    set_sourcedir("/doc/ares/concord")

    on_fetch(function (package)
        local result = {}
        result.includedirs = {"/doc/ares/concord/include"}
        return result
    end)
package_end()

-- Define graphix package (from local path)
package("graphix")
    set_kind("library", {headeronly = true})
    set_sourcedir("/doc/ares/graphix")

    on_fetch(function (package)
        local result = {}
        result.includedirs = {"/doc/ares/graphix/include"}
        return result
    end)
package_end()

-- Define optinum package (from local path - dependency of concord)
package("optinum")
    set_kind("library", {headeronly = true})
    set_sourcedir("/doc/ares/optinum")

    on_fetch(function (package)
        local result = {}
        result.includedirs = {"/doc/ares/optinum/include"}
        return result
    end)
package_end()

-- Define rerun_sdk package (from ~/.local installation)
package("rerun_sdk")
    set_kind("library", {headeronly = false})

    on_fetch(function (package)
        local home = os.getenv("HOME")
        if not home then
            return
        end

        local result = {}
        -- Link in correct order: rerun_sdk -> rerun_c -> arrow
        result.links = {"rerun_sdk", "rerun_c__linux_x64", "arrow", "arrow_bundled_dependencies"}
        result.linkdirs = {path.join(home, ".local/lib")}
        result.includedirs = {path.join(home, ".local/include")}

        -- Check if library exists
        local libpath = path.join(home, ".local/lib/librerun_sdk.a")
        if os.isfile(libpath) then
            return result
        end
    end)

    on_install(function (package)
        -- Already installed in ~/.local, nothing to do
        local home = os.getenv("HOME")
        package:addenv("PATH", path.join(home, ".local/bin"))
    end)
package_end()

-- Add required packages (NO MORE BOOST!)
add_requires("datapod")
add_requires("concord")
add_requires("graphix")
add_requires("optinum")

if has_config("examples") then
    add_requires("rerun_sdk")
end

if has_config("tests") then
    add_requires("doctest")
end

-- Main library target
target("farmtrax")
    set_kind("static")

    -- Add source files
    add_files("src/farmtrax/**.cpp")

    -- Add header files (including temp/)
    add_headerfiles("include/(farmtrax/**.hpp)")
    add_headerfiles("include/(temp/**.hpp)")
    add_includedirs("include", {public = true})

    -- Link dependencies (NO MORE BOOST!)
    add_packages("datapod", "concord", "graphix", "optinum")

    -- Conditional rerun support (only if package is found)
    if has_config("examples") then
        on_load(function (target)
            if target:pkg("rerun_sdk") then
                target:add("defines", "HAS_RERUN")
            end
        end)
        add_packages("rerun_sdk")
    end

    -- Set install files
    add_installfiles("include/(farmtrax/**.hpp)")
    add_installfiles("include/(temp/**.hpp)")
    on_install(function (target)
        local installdir = target:installdir()
        os.cp(target:targetfile(), path.join(installdir, "lib", path.filename(target:targetfile())))
    end)
target_end()

-- Examples (only build when farmtrax is the main project)
if has_config("examples") and os.projectdir() == os.curdir() then
    for _, filepath in ipairs(os.files("examples/*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("farmtrax")
            add_packages("datapod", "concord", "graphix", "optinum", "rerun_sdk")

            -- Add HAS_RERUN define for examples
            on_load(function (target)
                if target:pkg("rerun_sdk") then
                    target:add("defines", "HAS_RERUN")
                end
            end)

            add_includedirs("include")
        target_end()
    end
end

-- Tests (only build when farmtrax is the main project)
if has_config("tests") and os.projectdir() == os.curdir() then
    for _, filepath in ipairs(os.files("test/*.cpp")) do
        local filename = path.basename(filepath)
        target(filename)
            set_kind("binary")
            add_files(filepath)
            add_deps("farmtrax")
            add_packages("datapod", "concord", "graphix", "optinum", "doctest")
            add_includedirs("include")
            add_defines("DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN")

            -- Add as test
            add_tests("default", {rundir = os.projectdir()})
        target_end()
    end
end

-- Task to generate CMakeLists.txt
task("cmake")
    on_run(function ()
        import("core.project.config")

        -- Load configuration
        config.load()

        -- Generate CMakeLists.txt
        os.exec("xmake project -k cmakelists")

        print("CMakeLists.txt generated successfully!")
    end)

    set_menu {
        usage = "xmake cmake",
        description = "Generate CMakeLists.txt from xmake.lua",
        options = {}
    }
task_end()
