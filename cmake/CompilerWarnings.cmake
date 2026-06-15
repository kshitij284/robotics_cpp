function(target_set_warnings target access)
    set(CLANG_WARNINGS
        -Wall -Wextra -Wpedantic
        -Wshadow               # local var shadows outer scope
        -Wnon-virtual-dtor     # base class virtual methods, no virtual dtor
        -Wold-style-cast       # C-style casts
        -Wcast-align           # misaligned pointer cast
        -Wunused
        -Woverloaded-virtual   # derived class hides base virtual
        -Wconversion           # implicit type narrowing
        -Wsign-conversion
        -Wdouble-promotion     # float silently promoted to double
        -Wformat=2
        -Wimplicit-fallthrough
    )
    set(GCC_WARNINGS
        ${CLANG_WARNINGS}
        -Wmisleading-indentation
        -Wduplicated-cond
        -Wduplicated-branches
        -Wlogical-op
        -Wnull-dereference
    )

    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(${target} ${access} ${CLANG_WARNINGS})
    elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        target_compile_options(${target} ${access} ${GCC_WARNINGS})
    endif()
endfunction()
