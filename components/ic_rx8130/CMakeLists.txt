set(srcs
                "rx8130.cpp"
)

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES "driver"
                       PRIV_REQUIRES "i2c_bus")

