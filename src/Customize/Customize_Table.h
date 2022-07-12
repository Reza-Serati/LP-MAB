#ifndef __C_TABLE_H__
#define __C_TABLE_H__


#include <omnetpp.h>
#include <vector>
#include "../LoRa/LoRaMacFrame_m.h"

void cal_prob(char status, bool print);
void cal_weight(int rcvd, int prev_rcvd);
namespace flora{
class Customize_Table{
public:
    #define payload_size 5
    #define preamble_size 8

    #define init_SF_list 7
    #define init_CR_list 4
    #define init_CF_list 868
    #define init_BW_list 125
    #define init_TP_list 2

    int SF_list_size = 6; //SF: 7,8,9,10,11,12
    int CR_list_size = 1; //CR: 5,6,7,8p
    int CF_list_size = 1; //CF: 868,869,870 MHz
    int BW_list_size = 1; //BW: 125,250,500 KHz
    int TP_list_size = 5; //TP: 2,5,8,11,14 dbm
    int RSSI_table_size = SF_list_size*CR_list_size*BW_list_size*TP_list_size*CF_list_size;


    #define MIN(a,b) ((a) < (b) ? (a) : (b))
    int pow(int x, int y){
        int tmp = 1;
        for(int i=0; i<y; i++)
            tmp *= x;
        return tmp;
    }
    char* int_to_bit_string(int num, int bit_count){
        char* str = new char[bit_count];
        for (int i=bit_count-1, j=0; i>=0; i--, j++){
            if(num/(pow(2, i)) != 0){
                str[j] = '1';
                num -= pow(2, i);
            }
            else
                str[j] = '0';
        }
        return str;
    }
    int bit_string_to_int(char* str, int bit_count){
        int num = 0;
        for(int i=0, j=bit_count-1; i<bit_count; i++, j--){
            if((str[i] - '0') != 0)
                num += pow(2, j);
        }
        return num;
    }
    char* merge_bit_str(char* str1, char* str2, int bit_count1, int bit_count2){
        for(int i=bit_count1, j=0; i<bit_count1+bit_count2; i++, j++){
            str1[i] = str2[j];
        }
        return str1;
    }
    int* split_bit_str(char* str, int bit_count){
        int* ar = new int[2];
        char* temp = new char[bit_count];
        for(int i=0; i<bit_count; i++)
            temp[i] = str[i];
        ar[0] = bit_string_to_int(temp, bit_count);

        for(int i=0; i<bit_count; i++)
            temp[i] = str[i+bit_count];
        ar[1] = bit_string_to_int(temp, bit_count);

        return ar;
    }

    int n_payload(int sf, int cr, int pl){
        int bo = ceil( ((8 * pl) + (-4*sf) + 28 + 16) / (4*(sf-2)) ) * (cr+4);
        return 8 + std::max(bo, 0);
    }
    double t_payload(int sf, int bw, int cr, int pl){
        return n_payload(sf, cr, pl) * pow(2, sf) / bw;
    }
    double t_preamble(int sf, int bw, int pr){
        return (4.25 + pr) * pow(2,sf) / bw;
    }
    double cal_time(int sf, int bw, int cr, int pl, int pr){
    //    return floor(t_payload(sf, bw, cr, pl) + t_preamble(sf, bw, pr));
        return (t_payload(sf, bw, cr, pl) + t_preamble(sf, bw, pr));
    }
    double cal_energy(int sf, int bw, int cr, int tp, int cf){
        return cal_time(sf, bw, cr, payload_size, preamble_size) * tp + cf/100000;
    }

    struct RSSI_table_cell{
        int sf;
        int tp;
        int cr;
        int cf;
        int bw;
        units::values::Hz loRaBW;
        units::values::Hz loRaCF;
        double energy;
        std::string lable;
        int idx; // index in rssi_table

        ///////Beni////////////////
        double weight = 1;
        double prob = 0;
        int num_node_sent = 0;
        int num_node_rcvd = 0;
        int num_server_rcvd = 0;
        int num_server_sent = 1;

    };
    struct temp_known_node{
        MacAddress addr;
        int floor;
        int ceil;
        RSSI_table_cell rtc;
    };
    struct less_than_key{
        inline bool operator() (const RSSI_table_cell& rtc1, const RSSI_table_cell& rtc2){
            return (rtc1.energy < rtc2.energy);
        }
    };
    temp_known_node tkn;
//    std::vector<temp_known_node> knownNodes_mine;
    std::vector<RSSI_table_cell> RSSI_table;
    bool list_done = false;

    ////////////////Repli//////////////////////////////////
    int repli_max_interval = 5;
//    int repli_sending_interval_time = 1;
//    int repli_interval_count = 0;
//    bool repli_retransmiting = false;



    //////////////////////////////////////////////////////
    ////////////////Beni//////////////////////////////////
    double epsilon = 2.71;
    int T = 12*24*60*60;
    int K = RSSI_table_size;
    double gama = MIN(1, sqrt((K*log(K)) / ((epsilon-1) * T)));
    ////exploration///
    int prev_sent_idx = 0; //for exploration phase
    bool explr_started = false;
    int explr_itr_lim = 5;
    int explr_itr_cnt = 0; //counts the number of iteration in explr
    int explr_ttl_cnt = 1; //counts the total number of times pckt sent or rcvd in explr
                           // it starts at one bcs at first iteration it counts one number less
    int max_explr_days = 3;
    bool more_than_one_exploration = false;
    int RSSI_table_at_least_one_recvd_cnt = 0;
    bool RSSI_table_at_least_one_recvd_array[30][5+1];//TODO
    ////explotaion ////////////
    double learning_rate = 0;
    bool is_explr = true, is_explt = !is_explr;
    int rcvd_idx = 0;
    int explotation_itr = 100;
    int alpha = 1;
    int node_total_pckt_sent = 0;
    int end_node_sending_interval = 0;

    ////////////////////////////////////////////////////////////////////////////////

    std::vector<int> weighting(){
        std::vector<int> weight;
        int approach = 2;
        if (approach == 1){
            int section = TP_list_size;
            for (int i=0; i<RSSI_table_size; i++){
                weight.push_back(section - ceil(i / (RSSI_table_size / section)));
            }
        }

        else if(approach == 2){
            const int section = TP_list_size;
            float percentage [section];
            for(int i=0; i<section; i++){
                float total_rcvd = 0;
                for (int j=0; j<RSSI_table_size / section; j++){
                    total_rcvd += RSSI_table.at(j + (i*RSSI_table_size / section)).num_server_rcvd;
                }
                float coef = ((100/(section-1))*(i+1));
                percentage[i] = total_rcvd / section * 100;
//                std::cout<<total_rcvd << "\t" << total_sent << "\t" << percentage[i] << "\t" << coef << "\n";
                if(percentage [i] >= coef){
                    for (int j=0; j<RSSI_table_size / section; j++)
                        weight.push_back(section - i);
//                    std::cout<<"KIR KHAR";
                }
                else{
                    for (int j=0; j<RSSI_table_size / section; j++)
                        weight.push_back(1);
                }
            }
//            RSSI_table.at(45124);
        }

        return weight;
    }
    int counterrrr = 0;
    int select_nxt_idx(){
//        std::cout<<"\nSelecting Next Index\n";
        std::vector<RSSI_table_cell> rt;
        struct wght_rand_cell{
            double range;
            int idx;
        };
        double max_value = -1;
        int max_idx = int(rand() % RSSI_table_size);
        for(int i=0; i<RSSI_table_size; i++){
            if(RSSI_table.at(i).prob >= max_value){
                max_value = RSSI_table.at(i).prob;
                max_idx = i;
            }
        }
        ///////////////////////////////////////////////////////////////////////////////////////////////
//        counterrrr++;
//        if(counterrrr < 5){
//            std::cout<<"MAX_IDX " << max_idx <<"\n";
//            return max_idx;
//        }
//        counterrrr = 0;
//        else{
//            counterrrr = 0;
//            int randdd = int(rand() % RSSI_table_size);
//            std::cout<<"randdd " << randdd <<"\n";
//            return randdd;
//        }
        ///////////////////////////////////////////////////////////////////////////////////////////////
//        std::cout<<"MAX_VALUE:" << max_value <<std::endl;
        std::vector<wght_rand_cell> wght_rand_vect;

        int sum_of_weight=0;
        double rand_range=0;
        // for having lower power consumption, the lower indexes have higher weights
        std::vector<int> tmp_weight  = weighting();
        for(int i=0; i<RSSI_table_size; i++) {
            if(RSSI_table.at(i).prob >= 0.85 * (max_value) && max_idx != i){

                wght_rand_cell wrc;
//                if (RSSI_table.at(i).num_server_sent == 0)
//                    wrc.range = RSSI_table.at(i).prob * 1;
//                else

                wrc.range = RSSI_table.at(i).prob * RSSI_table.at(i).num_server_rcvd * tmp_weight.at(i)
                        *RSSI_table.at(i).num_server_sent;
//                    wrc.range = RSSI_table.at(i).prob * RSSI_table.at(i).num_node_sent;
                rand_range += wrc.range;
                wrc.idx = i;
                rt.push_back(RSSI_table.at(i));
                wght_rand_vect.push_back(wrc);
            }
        }
        double t  = std::fmod(rand(), rand_range );
//        std::cout << rand_range << "\n";
//        for (int i=0; i<15; i++)
//                std::cout << std::fmod(rand(), rand_range ) << "\t";
//        std::cout<<"\nRandom: " << t << std::endl;
        double tt = 0;
        for(int i=0; i<wght_rand_vect.size(); i++) {
            tt += wght_rand_vect.at(i).range;
            if(tt>=t){
//                std::cout<<"\nSelected Idx: " << RSSI_table.at(wght_rand_vect.at(i).idx).lable << std::endl;
                return wght_rand_vect.at(i).idx;
            }
        }
        return int(rand() % RSSI_table_size);

    }
    void cal_weight(int rcvd, int prev_rcvd, bool print){
        int approach = 3;
        double reward = 0;
        if (approach == 1){
            if (rcvd == prev_rcvd){
                reward = 1;
            }
            else{
                reward = 0;
            }
        }
        else if (approach == 2){
            int a = 3;
            double b = 0.3;
            int c = 1;
            for(int i=0; i<RSSI_table_size; i++)
                if (RSSI_table.at(i).prob == 0)
                    c++;
            if (rcvd == prev_rcvd){
                reward = (1 - ((RSSI_table.at(prev_rcvd).num_node_sent % a) * b)) / c;
            }
            else{
                reward = 0;
            }
        }
        else if (approach == 3){
            if (rcvd == prev_rcvd){
                if(RSSI_table.at(rcvd).tp == 2)
                    reward = 1.0;
                else if(RSSI_table.at(rcvd).tp == 5)
                    reward = 0.8;
                else if(RSSI_table.at(rcvd).tp == 8)
                    reward = 0.6;
                else if(RSSI_table.at(rcvd).tp == 11)
                    reward = 0.4;
                else if(RSSI_table.at(rcvd).tp == 14)
                    reward = 0.2;
            }
            else{
                reward = 0;
            }
        }
        double wt = RSSI_table.at(prev_rcvd).weight;
        double pt = RSSI_table.at(prev_rcvd).prob;
//        pt = 1;
        int K = RSSI_table_size;
        RSSI_table.at(prev_rcvd).weight = (wt * exp(((gama * reward) / (K*pt))));
//        std::cout<<"WEIGHT:" << RSSI_table.at(prev_rcvd).weight <<std::endl;

        if(print){
            std::cout<<"\n######################Server Result######################";
            for (int i=0; i<RSSI_table_size;i++){
                if(i%1 == 0)
                    std::cout<<endl;
                std::cout<< i << ": ";
                std::cout<<RSSI_table.at(i).lable << ":\t";
                std::cout<< "Prob " << RSSI_table.at(i).prob << "\t";
                std::cout<< "Wegh " << RSSI_table.at(i).weight << "\t";
                std::cout<< "nSen " << RSSI_table.at(i).num_node_sent << "\t";
                std::cout<< "SRec " << RSSI_table.at(i).num_server_rcvd << "\t";
                std::cout<< "SSen " << RSSI_table.at(i).num_server_sent << "\t";
            }
            std::cout<<endl;
        }
    }
    void cal_prob(){
        if(is_explr && false){
            int sent = 0, rcvd = 0;
            for (int i=0; i<RSSI_table_size;i++){
                rcvd += RSSI_table.at(i).num_server_rcvd;
                if (RSSI_table.at(i).num_server_rcvd != 0)
                    sent += RSSI_table.at(i).num_node_sent;
            }
            double tmp = rcvd;
            for (int i=0; i<RSSI_table_size;i++){
                if (RSSI_table.at(i).num_node_sent != 0)
                    RSSI_table.at(i).prob = ((double) RSSI_table.at(i).num_server_rcvd /
                            (double) RSSI_table.at(i).num_node_sent) / tmp;
            }
        }
        else if (is_explt || true){
            double temp = 0;
            int prob_zero = 0;
            int max_value = 0;

            for (int i=0; i<RSSI_table_size;i++){
                if(RSSI_table.at(i).num_server_rcvd != 0)
                    temp += RSSI_table.at(i).weight;
            }
            double wt;
            double normalize_sum = 0;
            int K = RSSI_table_size;
            for (int i=0; i<RSSI_table_size;i++){
                if(RSSI_table.at(i).num_server_rcvd != 0){
                    wt = RSSI_table.at(i).weight;
                    RSSI_table.at(i).prob = (((1-gama) * (wt/(temp))) + (gama/K));
                    normalize_sum += RSSI_table.at(i).prob;
                }
            }
            for (int i=0; i<RSSI_table_size;i++)
                RSSI_table.at(i).prob /= normalize_sum;
        }
    }

    int Hz_to_int(units::values::Hz in, char unit){
        if (unit == 'k')
            return atoi(inet::units::values::unit2string(in).c_str()) * 1000;
        else if (unit == 'm')
            return atoi(inet::units::values::unit2string(in).c_str()) * 1000000;
        else
            return atoi(inet::units::values::unit2string(in).c_str());
    }
    units::values::Hz int_to_Hz(int in, char unit){
        if(unit=='k')
            return inet::units::values::kHz(in);
        else if(unit=='m')
            return inet::units::values::MHz(in);
        return inet::units::values::Hz(in);
    }

    void exploraton_done(MacAddress addrr, bool stat){
//        if (RSSI_table.at(RSSI_table_size-1).num_node_sent == explr_itr_lim){
            MacAddress add (0x0AAA00000001);
            is_explr = !stat;
            is_explt = !is_explr;
//            if (add == addrr)
//                std::cout<<"KIRRRRRR\n";
//        }
    }
    int get_rssi_table_cell_index(int sf, int tp, int bw, int cr, int cf){
//        std::cout<< "SF: " << sf << " TP: " << tp << " BW: " << bw
//                << " CR: " << cr << " CF: " << cf << endl;
        for(int i=0; i<RSSI_table_size; i++){
            if (RSSI_table.at(i).sf == sf)
                if (RSSI_table.at(i).bw == bw)
                    if (RSSI_table.at(i).cr == cr)
                        if (RSSI_table.at(i).tp == tp)
                            if (RSSI_table.at(i).cf == cf)
                                return i;
        }
        std::cout<<"\n****Custom Table Index Not Found (get_rssi_table_cell_index() )\n";
        return -1;
    }


    void init_table(){
//        max_explr_days = ceil((end_node_sending_interval * RSSI_table_size * explr_itr_lim) / (24 * 60 * 60)) + 1;
//        std::cout << (end_node_sending_interval * RSSI_table_size * explr_itr_lim) << "\n";
//        std::cout << float(end_node_sending_interval * RSSI_table_size * explr_itr_lim / 24 / 60 / 60) << "\n";
//        std::cout << ceil((end_node_sending_interval * RSSI_table_size * explr_itr_lim) / (24 * 60 * 60)) << "\n";
//        max_explr_days = 3;

            for (int i=0; i<SF_list_size; i++){
                    for (int j=0; j<CR_list_size; j++){
                        for (int k=0; k<BW_list_size; k++){
                            for (int l=0; l<TP_list_size; l++){
                                for (int m=0; m<CF_list_size; m++){
                                    RSSI_table_cell rtc;

                                    rtc.sf = init_SF_list + i;
                                    rtc.cr = init_CR_list + j;
                                    rtc.cf = init_CF_list+(m*1);
                                    rtc.loRaCF = int_to_Hz(rtc.cf, 'm');
                                    rtc.bw = init_BW_list*(k+1);
                                    if (k==2) rtc.bw = 500;
                                    rtc.loRaBW = int_to_Hz(rtc.bw, 'k');
                                    rtc.tp = init_TP_list + 3*l;

                                    rtc.energy = cal_energy(rtc.sf, rtc.bw, rtc.cr, rtc.tp, rtc.cf);
                                    rtc.lable = std::to_string(rtc.sf) + "-" + std::to_string(rtc.tp)
                                            + "-" + std::to_string(rtc.bw) +
                                             "-" + std::to_string(rtc.cf) +"-" + std::to_string(rtc.cr);

                                    RSSI_table.push_back(rtc);
                                }
                            }
                        }
                    }
                }
                sort(RSSI_table.begin(), RSSI_table.end(), less_than_key());
//                for (int i=0;i<RSSI_table_size; i++){
//                    if(i%3 == 0)
//                        std::cout<<endl;
//                    std::cout << i << ". "<< RSSI_table[i].lable<<"\t";
//                }
//                std::cout<<endl;
//                std::cout<< "\nTable Created with " << RSSI_table_size << " Item\n";
//                print_rssi_table();
    }
    void print_rssi_table(){
        for (int i=0;i<RSSI_table_size; i++){
            if(i%1 == 0)
                std::cout<<endl;
            std::cout << i << ". "<< RSSI_table[i].lable<<"\t";
        }
        std::cout<<endl;
//                std::cout<< "\nTable Created with " << RSSI_table_size << " Item\n";
//                print_rssi_table();
    }

};
}
#endif
