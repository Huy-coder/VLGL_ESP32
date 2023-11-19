SET(SOURCES screens/ui_Screen1.c
    screens/ui_Screen2.c
    screens/ui_Screen3.c
    screens/ui_Screen5.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_1016388178.c
    images/ui_img_2016198774.c
    images/ui_img_483416384.c
    images/ui_img_325661808.c
    images/ui_img_1134473259.c
    images/ui_img_798946583.c
    images/ui_img_582018047.c)

add_library(ui ${SOURCES})
