// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.0
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"

void ui_scrRules_screen_init(void)
{
    ui_scrRules = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_scrRules, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_Panel12 = lv_obj_create(ui_scrRules);
    lv_obj_set_width(ui_Panel12, 797);
    lv_obj_set_height(ui_Panel12, 42);
    lv_obj_set_x(ui_Panel12, -1);
    lv_obj_set_y(ui_Panel12, 215);
    lv_obj_set_align(ui_Panel12, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel12, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Panel12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Panel12, lv_color_hex(0x5A595A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Button12 = lv_btn_create(ui_Panel12);
    lv_obj_set_width(ui_Button12, 136);
    lv_obj_set_height(ui_Button12, 54);
    lv_obj_set_x(ui_Button12, -328);
    lv_obj_set_y(ui_Button12, 0);
    lv_obj_set_align(ui_Button12, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button12, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button12, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_Button12, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_Button12, lv_color_hex(0x00AAF6), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button12, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblLock5 = lv_label_create(ui_Button12);
    lv_obj_set_width(ui_lblLock5, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblLock5, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblLock5, 1);
    lv_obj_set_y(ui_lblLock5, 0);
    lv_obj_set_align(ui_lblLock5, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblLock5, "Return");
    lv_obj_set_style_text_color(ui_lblLock5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_lblLock5, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_lblLock5, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_lblRuleList = lv_label_create(ui_scrRules);
    lv_obj_set_width(ui_lblRuleList, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_lblRuleList, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_lblRuleList, -22);
    lv_obj_set_y(ui_lblRuleList, -218);
    lv_obj_set_align(ui_lblRuleList, LV_ALIGN_CENTER);
    lv_label_set_text(ui_lblRuleList, "RULE LIST");
    lv_obj_set_style_text_font(ui_lblRuleList, &lv_font_montserrat_26, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TextArea2 = lv_textarea_create(ui_scrRules);
    lv_obj_set_width(ui_TextArea2, 769);
    lv_obj_set_height(ui_TextArea2, LV_SIZE_CONTENT);    /// 387
    lv_obj_set_x(ui_TextArea2, 3);
    lv_obj_set_y(ui_TextArea2, -12);
    lv_obj_set_align(ui_TextArea2, LV_ALIGN_CENTER);
    lv_textarea_set_text(ui_TextArea2,
                         "Rule1 - Always Use Explicit Waits Instead of Sleep\nRule2 - Use Headless Mode for Faster Execution\nRule3 - Use Headless Mode for Faster Execution\nRule4 - Use Headless Mode for Faster Execution\nRule5 - Always Use Explicit Waits Instead of Sleep\nRule6 - Avoid Hardcoded Waits & Optimize Locators\nRule7 - Always Use Explicit Waits Instead of Sleep\nRule8 - Use Headless Mode for Faster Execution\nRule9 - Use Headless Mode for Faster Execution");
    lv_textarea_set_placeholder_text(ui_TextArea2, "Placeholder...");
    lv_textarea_set_one_line(ui_TextArea2, true);
    lv_obj_set_style_text_color(ui_TextArea2, lv_color_hex(0x00667E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_TextArea2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_TextArea2, &lv_font_montserrat_22, LV_PART_MAIN | LV_STATE_DEFAULT);



    lv_obj_add_event_cb(ui_Button12, ui_event_Button12, LV_EVENT_ALL, NULL);

}
