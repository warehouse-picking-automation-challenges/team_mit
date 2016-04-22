
from openpyxl import load_workbook
import copy
import os
import json

def toExlInd(row, col):
    return "%s%d" % (chr(ord('A')+col-1), row)

def getValue(table, row, col):
    return table[toExlInd(row, col)].value

def loadHeuristic(filename, offset, endrow, max_strategy_num=6):
    wb = load_workbook(filename = filename)

    h = wb['Heuristic']
    hdict = {}
    
    item_id = ''
    for i in range(offset[0], endrow):
        if h[toExlInd(i, 2)].value:
            obj_id = getValue(h, i, 2)
            #print '\nloading object:', obj_id
            hdict[obj_id] = {}
        
        pose_type = getValue(h, i, 4)
        strategy_list = []
        for j in range(max_strategy_num):
            tmp = getValue(h, i, 5+j)
            if tmp:
                strategy_list.append(tmp)
        
        hdict[obj_id][pose_type] = strategy_list
        #print '\t', pose_type, ':', strategy_list
    
    return hdict

succRate = {
"oreo_mega_stuf": 1,
"crayola_64_ct": 1,
"paper_mate_12_count_mirado_black_warrior": 0.5,
"mead_index_cards": 0.5,
"rolodex_jumbo_pencil_cup": 0,
"mark_twain_huckleberry_finn": 1,
"laugh_out_loud_joke_book": 1,
"sharpie_accent_tank_style_highlighters": 0.5,
"stanley_66_052": 0.5,
"expo_dry_erase_board_eraser": 1,
"champion_copper_plus_spark_plug": 0.5,
"feline_greenies_dental_treats": 1,
"kong_air_dog_squeakair_tennis_ball": 0.3,
"dr_browns_bottle_brush": 0.5,
"kong_duck_dog_toy": 0.5,
"kong_sitting_frog_dog_toy": 0.5,
"munchkin_white_hot_duck_bath_toy": 0.4,
"mommys_helper_outlet_plugs": 0.3,
"kyjen_squeakin_eggs_plush_puppies": 0.5,
"first_years_take_and_toss_straw_cup": 0.4,
"highland_6539_self_stick_notes": 0.5,
"safety_works_safety_glasses": 0.4,
"genuine_joe_plastic_stir_sticks": 1,
"cheezit_big_original": 1,
"elmers_washable_no_run_school_glue": 1
}

hardbadRate = {
"oreo_mega_stuf": 0,
"crayola_64_ct": 0,
"paper_mate_12_count_mirado_black_warrior": 0,
"mead_index_cards": 0,
"rolodex_jumbo_pencil_cup": 1,
"mark_twain_huckleberry_finn": 0,
"laugh_out_loud_joke_book": 0,
"sharpie_accent_tank_style_highlighters": 0,
"stanley_66_052": 0,
"expo_dry_erase_board_eraser": 0,
"champion_copper_plus_spark_plug": 0,
"feline_greenies_dental_treats": 0,
"kong_air_dog_squeakair_tennis_ball": 0,
"dr_browns_bottle_brush": 0,
"kong_duck_dog_toy": 0,
"kong_sitting_frog_dog_toy": 0,
"munchkin_white_hot_duck_bath_toy": 0,
"mommys_helper_outlet_plugs": 0,
"kyjen_squeakin_eggs_plush_puppies": 0,
"first_years_take_and_toss_straw_cup": 0.9,
"highland_6539_self_stick_notes": 0,
"safety_works_safety_glasses": 0,
"genuine_joe_plastic_stir_sticks": 0,
"cheezit_big_original": 0,
"elmers_washable_no_run_school_glue": 0
}

hardbadRateMulti = {
"oreo_mega_stuf": 0,
"crayola_64_ct": 0,
"paper_mate_12_count_mirado_black_warrior": 0,
"mead_index_cards": 0,
"rolodex_jumbo_pencil_cup": 0,
"mark_twain_huckleberry_finn": 0,
"laugh_out_loud_joke_book": 0,
"sharpie_accent_tank_style_highlighters": 0.9,
"stanley_66_052": 0,
"expo_dry_erase_board_eraser": 0,
"champion_copper_plus_spark_plug": 0,
"feline_greenies_dental_treats": 0,
"kong_air_dog_squeakair_tennis_ball": 0,
"dr_browns_bottle_brush": 0,
"kong_duck_dog_toy": 0,
"kong_sitting_frog_dog_toy": 0,
"munchkin_white_hot_duck_bath_toy": 0,
"mommys_helper_outlet_plugs": 1,
"kyjen_squeakin_eggs_plush_puppies": 0,
"first_years_take_and_toss_straw_cup": 0,
"highland_6539_self_stick_notes": 0,
"safety_works_safety_glasses": 0,
"genuine_joe_plastic_stir_sticks": 0,
"cheezit_big_original": 0,
"elmers_washable_no_run_school_glue": 0
}

def isHardThing(order):
    return hardbadRate[order["item"]]
    
def isHardThingMulti(order):
    return hardbadRateMulti[order["item"]]

def sortOrder(work_order, bin_contents_all):
    norder = len(work_order)
    # a selection sort
    for i in range(norder):
        for j in range(i+1, norder):
            order_i = work_order[i]
            order_j = work_order[j]
            ncontents_i = len(bin_contents_all[order_i["bin"]])
            ncontents_j = len(bin_contents_all[order_j["bin"]])
            
            if isHardThing(order_i) > isHardThing(order_j) or \
               (isHardThing(order_i) == isHardThing(order_j) and isHardThingMulti(order_i) > isHardThingMulti(order_j)) or \
               (isHardThing(order_i) == isHardThing(order_j) and isHardThingMulti(order_i) == isHardThingMulti(order_j) and ncontents_i > ncontents_j) or \
               (isHardThing(order_i) == isHardThing(order_j) and isHardThingMulti(order_i) == isHardThingMulti(order_j) and ncontents_i == ncontents_j and succRate[order_i["item"]] < succRate[order_j["item"]]):
                tmp_order = copy.deepcopy(work_order[i])
                work_order[i] = copy.deepcopy(work_order[j])
                work_order[j] = copy.deepcopy(tmp_order)
    return work_order
    
def displayOrder(work_order, bin_contents_all, highlight = None):
    
    f = open(os.environ['HOME']+'/Desktop/'+'sorted_work_order.html', 'w')
    f.write('<HTML><HEAD><TITLE></TITLE><META HTTP-EQUIV="refresh" CONTENT="2"></HEAD><BODY>')
    
    norder = len(work_order)
    #print "Sorted orders: "
    for i in range(norder):
        output = ("%d. " % (i+1)) + work_order[i]["bin"] + " " + work_order[i]["item"] + ('\n\t\t\t\t\t\tbinContentNum: %d' % len(bin_contents_all[work_order[i]["bin"]])) + (' succRate: %.1f' % succRate[work_order[i]["item"]]) + '\n'
        #print(output)
        if highlight is not None and i == highlight:
            output = '<b>' + output + '</b>'
        f.write(output.replace('\n', '<br />').replace('\t', '&nbsp;'*20))
    
    f.write('</BODY></HTML>')
        
