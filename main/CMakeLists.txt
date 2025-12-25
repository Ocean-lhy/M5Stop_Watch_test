idf_component_register(SRCS main.cpp tools.cpp power_management.cpp audio_user.cpp touch_user.cpp wakeup_test.cpp display_user.cpp imu_user.cpp ui.cpp
                    INCLUDE_DIRS "."
                    EMBED_FILES "demo.pcm"
                    )