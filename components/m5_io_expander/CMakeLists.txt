set(PI4IOE5V6408_SOURCES
    "components/m5_io_pi4ioe5v6408/m5_io_pi4ioe5v6408.cpp"
)

set(PY32IOEXPANDER_SOURCES
    "components/m5_io_py32ioexpander/m5_io_py32ioexpander.cpp"
)

idf_component_register(SRCS M5_io_expander.cpp ${PI4IOE5V6408_SOURCES} ${PY32IOEXPANDER_SOURCES}
                    INCLUDE_DIRS "." "components/m5_io_pi4ioe5v6408" "components/m5_io_py32ioexpander"
                    REQUIRES i2c_bus
                    REQUIRES driver
                    )