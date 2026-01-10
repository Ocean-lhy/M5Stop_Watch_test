idf_component_register(SRCS main.cpp tools.cpp power_management.cpp audio_user.cpp touch_user.cpp wakeup_test.cpp display_user.cpp imu_user.cpp ui.cpp wifi_manager.cpp fft.cpp
                    INCLUDE_DIRS "."
                    EMBED_FILES "demo.pcm" "tone.wav" "voice_start.wav" "voice_button.wav" "voice_touch.wav" "hotel_california.wav" "DEAR_1.wav" "DEAR_2.wav"
                    )