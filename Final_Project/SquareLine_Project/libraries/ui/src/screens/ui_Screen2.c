// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.3
// Project name: SquareLine_Project

#include "../ui.h"

void ui_Screen2_screen_init(void)
{
ui_Screen2 = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Screen2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Button5 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button5, 55);
lv_obj_set_height( ui_Button5, 35);
lv_obj_set_x( ui_Button5, 210 );
lv_obj_set_y( ui_Button5, 115 );
lv_obj_set_align( ui_Button5, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button5, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Button5, lv_color_hex(0xFE0303), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button5, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Checkbox1 = lv_checkbox_create(ui_Screen2);
lv_checkbox_set_text(ui_Checkbox1,"All motors");
lv_obj_set_width( ui_Checkbox1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Checkbox1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Checkbox1, -172 );
lv_obj_set_y( ui_Checkbox1, 107 );
lv_obj_set_align( ui_Checkbox1, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Checkbox1, LV_STATE_CHECKED );     /// States
lv_obj_add_flag( ui_Checkbox1, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags

ui_Slider1 = lv_slider_create(ui_Screen2);
lv_slider_set_value( ui_Slider1, 50, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Slider1)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Slider1, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Slider1, 150);
lv_obj_set_height( ui_Slider1, 10);
lv_obj_set_x( ui_Slider1, -65 );
lv_obj_set_y( ui_Slider1, -94 );
lv_obj_set_align( ui_Slider1, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Slider1, LV_STATE_DISABLED );     /// States


ui_Slider2 = lv_slider_create(ui_Screen2);
lv_slider_set_value( ui_Slider2, 50, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Slider2)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Slider2, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Slider2, 150);
lv_obj_set_height( ui_Slider2, 10);
lv_obj_set_x( ui_Slider2, -65 );
lv_obj_set_y( ui_Slider2, -44 );
lv_obj_set_align( ui_Slider2, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Slider2, LV_STATE_DISABLED );     /// States


ui_Slider3 = lv_slider_create(ui_Screen2);
lv_slider_set_value( ui_Slider3, 50, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Slider3)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Slider3, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Slider3, 150);
lv_obj_set_height( ui_Slider3, 10);
lv_obj_set_x( ui_Slider3, -65 );
lv_obj_set_y( ui_Slider3, 4 );
lv_obj_set_align( ui_Slider3, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Slider3, LV_STATE_DISABLED );     /// States


ui_Slider4 = lv_slider_create(ui_Screen2);
lv_slider_set_value( ui_Slider4, 50, LV_ANIM_OFF);
if (lv_slider_get_mode(ui_Slider4)==LV_SLIDER_MODE_RANGE ) lv_slider_set_left_value( ui_Slider4, 0, LV_ANIM_OFF);
lv_obj_set_width( ui_Slider4, 150);
lv_obj_set_height( ui_Slider4, 10);
lv_obj_set_x( ui_Slider4, -65 );
lv_obj_set_y( ui_Slider4, 54 );
lv_obj_set_align( ui_Slider4, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Slider4, LV_STATE_DISABLED );     /// States


ui_Label6 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label6, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label6, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label6, -190 );
lv_obj_set_y( ui_Label6, -94 );
lv_obj_set_align( ui_Label6, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label6,"Motor 1");

ui_Label7 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label7, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label7, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label7, -190 );
lv_obj_set_y( ui_Label7, -44 );
lv_obj_set_align( ui_Label7, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label7,"Motor 2");

ui_Label8 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label8, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label8, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label8, -190 );
lv_obj_set_y( ui_Label8, 4 );
lv_obj_set_align( ui_Label8, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label8,"Motor 3");

ui_Label9 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label9, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label9, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label9, -190 );
lv_obj_set_y( ui_Label9, 54 );
lv_obj_set_align( ui_Label9, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label9,"Motor 4");

ui_Switch1 = lv_switch_create(ui_Screen2);
lv_obj_set_width( ui_Switch1, 50);
lv_obj_set_height( ui_Switch1, 25);
lv_obj_set_x( ui_Switch1, 65 );
lv_obj_set_y( ui_Switch1, -94 );
lv_obj_set_align( ui_Switch1, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Switch1, LV_STATE_CHECKED | LV_STATE_DISABLED );     /// States


ui_Switch2 = lv_switch_create(ui_Screen2);
lv_obj_set_width( ui_Switch2, 50);
lv_obj_set_height( ui_Switch2, 25);
lv_obj_set_x( ui_Switch2, 65 );
lv_obj_set_y( ui_Switch2, -44 );
lv_obj_set_align( ui_Switch2, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Switch2, LV_STATE_CHECKED | LV_STATE_DISABLED );     /// States


ui_Switch3 = lv_switch_create(ui_Screen2);
lv_obj_set_width( ui_Switch3, 50);
lv_obj_set_height( ui_Switch3, 25);
lv_obj_set_x( ui_Switch3, 65 );
lv_obj_set_y( ui_Switch3, 4 );
lv_obj_set_align( ui_Switch3, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Switch3, LV_STATE_CHECKED | LV_STATE_DISABLED );     /// States


ui_Switch4 = lv_switch_create(ui_Screen2);
lv_obj_set_width( ui_Switch4, 50);
lv_obj_set_height( ui_Switch4, 25);
lv_obj_set_x( ui_Switch4, 65 );
lv_obj_set_y( ui_Switch4, 54 );
lv_obj_set_align( ui_Switch4, LV_ALIGN_CENTER );
lv_obj_add_state( ui_Switch4, LV_STATE_CHECKED | LV_STATE_DISABLED );     /// States


ui_Button9 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button9, 35);
lv_obj_set_height( ui_Button9, 33);
lv_obj_set_x( ui_Button9, 132 );
lv_obj_set_y( ui_Button9, -94 );
lv_obj_set_align( ui_Button9, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button9, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button9, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Button9, 90, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Button9, lv_color_hex(0x00FF21), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button9, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Button10 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button10, 35);
lv_obj_set_height( ui_Button10, 33);
lv_obj_set_x( ui_Button10, 132 );
lv_obj_set_y( ui_Button10, 54 );
lv_obj_set_align( ui_Button10, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button10, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button10, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Button10, 90, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Button10, lv_color_hex(0x00FF21), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button10, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Button11 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button11, 35);
lv_obj_set_height( ui_Button11, 33);
lv_obj_set_x( ui_Button11, 132 );
lv_obj_set_y( ui_Button11, 4 );
lv_obj_set_align( ui_Button11, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button11, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button11, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Button11, 90, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Button11, lv_color_hex(0x00FF21), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button11, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Button12 = lv_btn_create(ui_Screen2);
lv_obj_set_width( ui_Button12, 35);
lv_obj_set_height( ui_Button12, 33);
lv_obj_set_x( ui_Button12, 132 );
lv_obj_set_y( ui_Button12, -44 );
lv_obj_set_align( ui_Button12, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_Button12, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Button12, 90, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_Button12, lv_color_hex(0x00FF21), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Button12, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label12 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label12, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label12, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label12, -65 );
lv_obj_set_y( ui_Label12, -120 );
lv_obj_set_align( ui_Label12, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label12,"");

ui_Label13 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label13, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label13, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label13, -65 );
lv_obj_set_y( ui_Label13, -65 );
lv_obj_set_align( ui_Label13, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label13,"");

ui_Label14 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label14, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label14, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label14, -65 );
lv_obj_set_y( ui_Label14, 30 );
lv_obj_set_align( ui_Label14, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label14,"");

ui_Label15 = lv_label_create(ui_Screen2);
lv_obj_set_width( ui_Label15, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label15, LV_SIZE_CONTENT);   /// 1
lv_obj_set_x( ui_Label15, -65 );
lv_obj_set_y( ui_Label15, -20 );
lv_obj_set_align( ui_Label15, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label15,"");

lv_obj_add_event_cb(ui_Button5, ui_event_Button5, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Checkbox1, ui_event_Checkbox1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Slider1, ui_event_Slider1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Slider2, ui_event_Slider2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Slider3, ui_event_Slider3, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Slider4, ui_event_Slider4, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Switch1, ui_event_Switch1, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Switch2, ui_event_Switch2, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Switch3, ui_event_Switch3, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_Switch4, ui_event_Switch4, LV_EVENT_ALL, NULL);

}