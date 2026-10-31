# The Unity package. Assembles dist/com.rant.math/, which Unity adds from disk and the release pushes
# to the upm branch, from bindings/csharp/, the libraries in dist/native/ and the packed C. It carries
# no Rant.Types, since one project may hold only one assembly of that name, and declares no dependency:
# com.rant.rant carries the types, and a project with no Rant adds com.rant.types. Unity's immutable
# package cache synthesizes no .meta files, so every entry gets one here, with a GUID that is the md5
# of its path so an upgrade keeps references.
#   cmake -D DIST=<the folder holding rant_math.h> -P tools/unity.cmake

cmake_minimum_required(VERSION 3.18)

get_filename_component(ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
file(STRINGS "${ROOT}/VERSION" VERSION LIMIT_COUNT 1)
set(PKG com.rant.math)
set(OUT "${ROOT}/dist/${PKG}")
if(NOT DEFINED DIST)
  set(DIST "${ROOT}/build/dist")
endif()
if(NOT EXISTS "${DIST}/rant_math.h")
  message(FATAL_ERROR "unity: no ${DIST}/rant_math.h, build first or set DIST")
endif()

# LF whatever the host, so the upm branch is identical from every machine.
function(write_lf f text)
  file(CONFIGURE OUTPUT "${f}" CONTENT "${text}" @ONLY NEWLINE_STYLE LF)
endfunction()

set(META_FOLDER "folderAsset: yes
DefaultImporter:
  externalObjects: {}
  userData:
")
set(META_CS "MonoImporter:
  externalObjects: {}
  serializedVersion: 2
  defaultReferences: []
  executionOrder: 0
  icon: {instanceID: 0}
  userData:
")
set(META_ASMDEF "AssemblyDefinitionImporter:
  externalObjects: {}
  userData:
")
set(META_PACKAGE "PackageManifestImporter:
  externalObjects: {}
  userData:
")
set(META_WEBGL "PluginImporter:
  externalObjects: {}
  serializedVersion: 2
  iconMap: {}
  executionOrder: {}
  defineConstraints: []
  isPreloaded: 0
  isOverridable: 0
  isExplicitlyReferenced: 0
  validateReferences: 1
  platformData:
  - first:
      Any:
    second:
      enabled: 0
      settings: {}
  - first:
      Editor: Editor
    second:
      enabled: 0
      settings:
        DefaultValueInitialized: true
  - first:
      WebGL: WebGL
    second:
      enabled: 1
      settings: {}
  userData:
")
set(META_TEXT "TextScriptImporter:
  externalObjects: {}
  userData:
")

# A native plugin enabled for exactly one platform: the Editor on that OS and its standalone player.
# Three libraries share the name rant_math, so nothing may say Any.
function(plugin_meta out os)
  set(cpu_win None)
  set(cpu_linux None)
  set(cpu_osx None)
  if(os STREQUAL "Windows")
    set(cpu x86_64)
    set(cpu_win x86_64)
  elseif(os STREQUAL "Linux")
    set(cpu x86_64)
    set(cpu_linux x86_64)
  else()
    set(cpu AnyCPU)
    set(cpu_osx AnyCPU)
  endif()
  foreach(p win linux osx)
    if(cpu_${p} STREQUAL "None")
      set(on_${p} 0)
    else()
      set(on_${p} 1)
    endif()
  endforeach()
  set(${out} "PluginImporter:
  externalObjects: {}
  serializedVersion: 2
  iconMap: {}
  executionOrder: {}
  defineConstraints: []
  isPreloaded: 0
  isOverridable: 0
  isExplicitlyReferenced: 0
  validateReferences: 1
  platformData:
  - first:
      Any:
    second:
      enabled: 0
      settings: {}
  - first:
      Editor: Editor
    second:
      enabled: 1
      settings:
        CPU: ${cpu}
        DefaultValueInitialized: true
        OS: ${os}
  - first:
      Standalone: Linux64
    second:
      enabled: ${on_linux}
      settings:
        CPU: ${cpu_linux}
  - first:
      Standalone: OSXUniversal
    second:
      enabled: ${on_osx}
      settings:
        CPU: ${cpu_osx}
  - first:
      Standalone: Win64
    second:
      enabled: ${on_win}
      settings:
        CPU: ${cpu_win}
  userData:
" PARENT_SCOPE)
endfunction()

function(meta_body out rel)
  if(IS_DIRECTORY "${OUT}/${rel}")
    set(body "${META_FOLDER}")
  elseif(rel STREQUAL "package.json")
    set(body "${META_PACKAGE}")
  elseif(rel MATCHES "[.]cs$")
    set(body "${META_CS}")
  elseif(rel MATCHES "[.]asmdef$")
    set(body "${META_ASMDEF}")
  elseif(rel MATCHES "Plugins/win-")
    plugin_meta(body Windows)
  elseif(rel MATCHES "Plugins/linux-")
    plugin_meta(body Linux)
  elseif(rel MATCHES "Plugins/WebGL/.")
    set(body "${META_WEBGL}")
  elseif(rel MATCHES "Plugins/osx")
    plugin_meta(body OSX)
  else()
    set(body "${META_TEXT}")
  endif()
  set(${out} "${body}" PARENT_SCOPE)
endfunction()

file(REMOVE_RECURSE "${OUT}")
file(MAKE_DIRECTORY "${OUT}/Runtime/Plugins")
write_lf("${OUT}/package.json" "{
  \"name\": \"${PKG}\",
  \"version\": \"${VERSION}\",
  \"displayName\": \"Rant Math\",
  \"description\": \"Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds, with conversions to Unity's axes. Needs com.rant.rant, or com.rant.types with no Rant.\",
  \"unity\": \"2021.3\",
  \"keywords\": [\"robotics\", \"geometry\", \"poses\", \"pointcloud\", \"rant\"],
  \"author\": { \"name\": \"Nathan George\" }
}
")
write_lf("${OUT}/README.md" "# Rant Math

Math on Rant's standard types: poses, shapes, fits, cameras, images and point clouds.

\`\`\`csharp
using Rant.Math;
using Rant.Types;
using UnityEngine;

var pose = transform.position.ToRant();
\`\`\`

Add it beside \`com.rant.rant\`, or beside \`com.rant.types\` in a project with no Rant.
")
write_lf("${OUT}/Runtime/Rant.Math.Runtime.asmdef" "{
  \"name\": \"Rant.Math.Runtime\",
  \"rootNamespace\": \"Rant.Math\",
  \"references\": [\"Rant.Types\"],
  \"includePlatforms\": [],
  \"excludePlatforms\": [],
  \"allowUnsafeCode\": true,
  \"overrideReferences\": false,
  \"autoReferenced\": true,
  \"defineConstraints\": [],
  \"noEngineReferences\": false
}
")
file(COPY "${ROOT}/bindings/csharp/RantMath.cs" "${ROOT}/bindings/csharp/RantMath.g.cs" "${ROOT}/bindings/csharp/Conversions.cs"
     DESTINATION "${OUT}/Runtime")

# Unity targets these three desktops. win-arm64 and linux-arm64 would collide on the library name,
# and no standalone player exists for them.
foreach(entry "win-x64/rant_math.dll" "linux-x64/librant_math.so" "osx/librant_math.dylib")
  if(EXISTS "${ROOT}/dist/native/${entry}")
    get_filename_component(rid "${entry}" DIRECTORY)
    file(COPY "${ROOT}/dist/native/${entry}" DESTINATION "${OUT}/Runtime/Plugins/${rid}")
  else()
    message(STATUS "unity: no dist/native/${entry}, that platform gets no plugin")
  endif()
endforeach()

# A WebGL player compiles the C into its own wasm.
set(WG "${OUT}/Runtime/Plugins/WebGL")
file(MAKE_DIRECTORY "${WG}")
file(COPY "${DIST}/rant_math.h" "${DIST}/rant_math.c" DESTINATION "${WG}")

file(GLOB_RECURSE entries LIST_DIRECTORIES true RELATIVE "${OUT}" "${OUT}/*")
foreach(rel IN LISTS entries)
  string(MD5 guid "${PKG}/${rel}")
  meta_body(body "${rel}")
  write_lf("${OUT}/${rel}.meta" "fileFormatVersion: 2\nguid: ${guid}\n${body}")
endforeach()
message(STATUS "wrote ${OUT}")
