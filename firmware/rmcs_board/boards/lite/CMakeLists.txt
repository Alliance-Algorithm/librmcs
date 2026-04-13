sdk_inc(.)
sdk_src(board.c)
set_property(GLOBAL PROPERTY LIBRMCS_USB_PID 0xA801)
sdk_compile_definitions(LIBRMCS_USB_PID=0xA801)

if(LIBRMCS_BUILDING_FIRMWARE_APP)
    sdk_app_inc(app)
    sdk_app_src(app/board_app.cpp)
endif()
