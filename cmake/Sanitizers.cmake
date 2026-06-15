option(ENABLE_SANITIZERS "Enable ASan + UBSan" OFF)

function(target_enable_sanitizers target)
    if(NOT ENABLE_SANITIZERS)
        return()
    endif()

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
        set(SAN_FLAGS
            -fsanitize=address,undefined
            -fno-omit-frame-pointer      # better stack traces
            -fno-sanitize-recover=all    # crash on UB, don't continue silently
        )
        target_compile_options(${target} PRIVATE ${SAN_FLAGS})
        target_link_options(   ${target} PRIVATE ${SAN_FLAGS})
    else()
        message(WARNING "Sanitizers not supported on this compiler")
    endif()
endfunction()
