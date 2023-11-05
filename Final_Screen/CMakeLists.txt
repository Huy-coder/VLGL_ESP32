SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    screens/ui_Screen3.c
    screens/ui_Screen5.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_472371_png.c
    images/ui_img_568182_png.c)

add_library(ui ${SOURCES})
