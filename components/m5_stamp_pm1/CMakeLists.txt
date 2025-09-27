# CMakeLists.txt for M5 Stamp Timer Power2 Component

idf_component_register(
    SRCS 
        "src/m5_stamp_pm1.cpp"
    INCLUDE_DIRS 
        "src"
    REQUIRES 
        "espressif__i2c_bus"
    PRIV_REQUIRES
        ""
)

# Component version
set(COMPONENT_VERSION "0.0.1")

# # Add component-specific compile options if needed
# target_compile_options(${COMPONENT_LIB} PRIVATE
#     -Wno-unused-parameter
#     -Wno-missing-field-initializers
# )

# # Add preprocessor definitions
# target_compile_definitions(${COMPONENT_LIB} PRIVATE
#     PM1_COMPONENT_VERSION="${COMPONENT_VERSION}"
# )
