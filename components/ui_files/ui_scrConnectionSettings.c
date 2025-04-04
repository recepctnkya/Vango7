// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_scrConnectionSettings_screen_init(void)
{
    ui_scrConnectionSettings = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_scrConnectionSettings, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel6 = lv_obj_create(ui_scrConnectionSettings);
    lv_obj_set_width(ui_Panel6, 797);
    lv_obj_set_height(ui_Panel6, 42);
    lv_obj_set_x(ui_Panel6, -1);
    lv_obj_set_y(ui_Panel6, 215);
    lv_obj_set_align(ui_Panel6, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel6, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel6, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel6, lv_color_hex(0x5A595A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button13 = lv_btn_create(ui_Panel6);
    lv_obj_set_width(ui_Button13, 136);
    lv_obj_set_height(ui_Button13, 54);
    lv_obj_set_x(ui_Button13, -328);
    lv_obj_set_y(ui_Button13, 0);
    lv_obj_set_align(ui_Button13, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button13, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button13, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button13, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button13, lv_color_hex(0x00AAF6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button13, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblLock7 = lv_label_create(ui_Button13);
    lv_obj_set_width(ui_lblLock7, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblLock7, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblLock7, 1);
    lv_obj_set_y(ui_lblLock7, 0);
    lv_obj_set_align(ui_lblLock7, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblLock7, "Return");
    lv_obj_set_style_text_color(ui_lblLock7, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblLock7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblLock7, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_Button13, ui_event_Button13, LV_EVENT_ALL, NULL);

}
