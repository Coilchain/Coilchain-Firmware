#ifndef BCU_DISP_H
#define BCU_DISP_H

#include "common_inc.h"
#include <TFT_eSPI.h>
#include <SPI.h>

namespace BcuDisplay{

#define BCU_DISP_NUMS           11
#define BCU_DISP_PARAMS_SHIFT   6

#define TFT_MAX_W (320)
#define TFT_MAX_H (240)

enum BcuDispParams {
    BCU_DISP_PARAMS_KM  = 0,
    BCU_DISP_PARAMS_KE  = 1,
    BCU_DISP_PARAMS_KT  = 2,
    BCU_DISP_PARAMS_LF  = 3,
    BCU_DISP_PARAMS_MAX
};

typedef struct BcuDispNode{
    std::string node_name;
    std::string unit_name;
    bool is_params;
    int x;
    int y;
    int value;
    int limited_value;
    bool need_loop;
    int rank;
    int font_size;
} BcuDispNode_t;

typedef struct BcuDispConfig{
    pin_size_t button_up;
    pin_size_t button_down;
    pin_size_t button_mid;
    PinStatus button_isr_action;
    uint32_t color_bg;
    uint32_t color_font;
    uint32_t color_highlight;
    //Add config here!
} BcuDispConfig_t;

static BcuDispNode_t disp_node_table[BCU_DISP_NUMS] = {
    {"SPD", "kmh", false, 15, 10, 100, 100, false, 0,  3},
    {"PWR", "w", false, 15, 40, 100, 1000, false, 1,  3},
    {"CAD", "rpm", false, 15, 70, 100, 1000, false, 2, 3},
    {"LVL", "s", false, 15, 100, 100, 2000,false, 3,   3},
    {"Po", "w", false, 15, 160, 246, 1000, false, 4, 2},
    {"Pi", "w", false, 15,180, 250, 100, false, 5, 2},
    {"To", "nm", false, 15, 200, 122, 1000, false, 6, 2},
    {"Km", "", true,  170, 140, 8, 20, true, 7, 3},
    {"Ke", "", true,  170, 165, 5, 20, true, 8, 3},
    {"Kt", "", true,  170, 190, 3, 20, true, 9, 3},
    {"Lf", "", true,  170, 215, 10,20, true, 10, 3}
};

static int param_items = 0;
static int param_value = 0;

typedef std::pair<const std::string, BcuDispNode_t> DISP_NODE_TYPE;

class BcuDisp:public TFT_eSPI{
public:
    BcuDisp(TFT_eSPI& t_esp, BcuDispConfig bcu_disp_config): tft_esp(t_esp), bcu_disp_cfg(bcu_disp_config) {};
    virtual ~BcuDisp(){bcu_node_m.clear();};

    void init();
    void print();
    void collect(const std::string node_name, const int& value) {
        bcu_node_m[node_name].value =  std::min(bcu_node_m[node_name].limited_value, value);
    };
    int get(const std::string node_name) {
        return bcu_node_m[node_name].value;
    };

    void add_node(BcuDispNode_t disp_node) {
        bcu_node_m.insert(DISP_NODE_TYPE(disp_node.node_name, disp_node));
    };

    void delete_node(const std::string name) {
        bcu_node_m.erase(name);
    };

    BcuDispConfig_t bcu_disp_cfg;

private:
    void print_node(BcuDispNode_t& disp_node);
    static void notify_up() {
        param_items = (++param_items) > BCU_DISP_PARAMS_MAX ? BCU_DISP_PARAMS_KM: param_items;
    };

    static void notify_dn() {
        param_items = (--param_items) <= BCU_DISP_PARAMS_KM ? BCU_DISP_PARAMS_MAX: param_items;
    };

    static void notify_mid(){
        param_value++;
    };

    std::map<std::string, BcuDispNode_t> bcu_node_m;
    TFT_eSPI& tft_esp;
    int last_rank = BCU_DISP_PARAMS_SHIFT + 1;
};
}

#endif


