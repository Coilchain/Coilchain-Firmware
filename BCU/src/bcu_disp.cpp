#include "bcu_disp.h"

namespace BcuDisplay{
// Only for debug.
static void print_node_ss(const BcuDispNode_t& disp_node) {
    Serial.print(">> "); Serial.print("    \n");
    Serial.print("node_name: ");Serial.print(disp_node.node_name.c_str()); Serial.print("   \n");
    Serial.print("unit_name: ");Serial.print(disp_node.unit_name.c_str()); Serial.print("   \n");
    Serial.print("is_params: ");Serial.print(disp_node.is_params); Serial.print("   \n");
    Serial.print("x: ");Serial.print(disp_node.x); Serial.print("   \n");
    Serial.print("y: ");Serial.print(disp_node.y); Serial.print("   \n");
    Serial.print("value: ");Serial.print(disp_node.value); Serial.print("   \n");
    Serial.print("need_loop: ");Serial.print(disp_node.need_loop); Serial.print("   \n");
    Serial.print("rank: ");Serial.print(disp_node.rank); Serial.print("   \n");
    Serial.print("font_size: ");Serial.print(disp_node.font_size); Serial.print("   \n");

    return;
}

void BcuDisp::init() noexcept {

    bcu_node_m.clear();
    for(size_t i = 0; i < BCU_DISP_NUMS; i++) {
        BcuDispNode_t disp_node_tmp = disp_node_table[i];
        this->add_node(disp_node_tmp);
    }

    pinMode(bcu_disp_cfg.button_sel, INPUT);
    pinMode(bcu_disp_cfg.button_down, INPUT_PULLUP);
    pinMode(bcu_disp_cfg.button_up, INPUT_PULLUP);
    pinMode(bcu_disp_cfg.button_mid, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(bcu_disp_cfg.button_sel), &notify_sel,
                                          bcu_disp_cfg.button_sel_isr_action);
    attachInterrupt(digitalPinToInterrupt(bcu_disp_cfg.button_down), &notify_up,
                                          bcu_disp_cfg.button_isr_action);
    attachInterrupt(digitalPinToInterrupt(bcu_disp_cfg.button_up), &notify_dn,
                                          bcu_disp_cfg.button_isr_action);
    attachInterrupt(digitalPinToInterrupt(bcu_disp_cfg.button_mid), &notify_mid,
                                          bcu_disp_cfg.button_isr_action);

    tft_esp.init();
    tft_esp.setRotation(1);
    tft_esp.setTextSize(1);
    tft_esp.fillScreen(bcu_disp_cfg.color_bg);
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
}

void BcuDisp::print(void) noexcept {
    std::for_each(bcu_node_m.begin(), bcu_node_m.end(),
                [this](DISP_NODE_TYPE& m_element){
                    this->print_node(m_element.second);
                });
}

static inline int tft_write_safe(int x, int y, int w, int h) {
    if((x < 0) || (x + w > TFT_MAX_W)) {
        return -1;
    }

    if((x < 0) || (x + h > TFT_MAX_H)) {
        return -1;
    }

    return 0;
}

// HardCode, need to refix later.
static inline std::string format_value(const float& value) {
    std::stringstream buf;
    buf.precision(1);
    buf.setf(std::ios::fixed);
    buf << value;

    std::string ss =  buf.str();
    
    if(ss.size() == 3)
        return("  " + ss);
    if(ss.size() == 4)
        return(" " + ss);
    if(ss.size() == 5)
        return(ss);
}

void BcuDisp::print_node(BcuDispNode_t& disp_node) noexcept {

    tft_esp.setCursor(disp_node.x, disp_node.y);
    auto font_size = std::min(disp_node.font_size, 10);
    tft_esp.setTextSize(font_size);
    bool params_is_selected = disp_node.rank == (param_items + BCU_DISP_PARAMS_SHIFT) && disp_node.is_params;

    if(params_is_selected) {
        if((disp_node.value + param_value * disp_node.step_ratio) >= disp_node.value_max)
            disp_node.value = disp_node.value_max;
        else if((disp_node.value + param_value * disp_node.step_ratio) <= disp_node.value_min)
            disp_node.value = disp_node.value_min;
        else
            disp_node.value += param_value * disp_node.step_ratio;
        param_value = 0;

        if(disp_node.rank != last_rank) {
            param_value = 0;
        }

        last_rank =  disp_node.rank;
        tft_esp.setTextColor(bcu_disp_cfg.color_font, bcu_disp_cfg.color_highlight);
    }
    
    //need transfer float to .0 format.
    std::string disp_str = format_value(disp_node.value);
    std::string print_str = disp_node.node_name + ":" + disp_str + " " + disp_node.unit_name;

    if(tft_write_safe(disp_node.x, disp_node.y,
                      print_str.length() * disp_node.font_size,
                      disp_node.font_size) != 0) {
        return;
    };
    
    tft_esp.print(print_str.c_str());
    #ifdef OPEN_BCU_DISP_DEBUG
    print_node_ss(disp_node);
    #endif

    if(params_is_selected)
        tft_esp.setTextColor(bcu_disp_cfg.color_font, bcu_disp_cfg.color_bg);
}
} //end namespace