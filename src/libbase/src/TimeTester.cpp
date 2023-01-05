#include "libbase/common.h"

namespace wmj{
    TimeTester::TimeTester(){
        m_color_info = std::vector<std::string>{"\033[31m", "\033[1;34m", "\033[1;33m", "\033[1;35m", "\033[1;37m"};
    }

    double TimeTester::now(){
        timeval tv;
        gettimeofday(&tv, NULL);
        return (double)tv.tv_sec + (double)tv.tv_usec / 1000.;
    }
    
    bool TimeTester::start(std::string label){
        if(label.empty()) return false;
        std::vector<std::string> str_vec;
        if(split(label, str_vec)){
            Time current_time;
            current_time.start = now();
            auto it = m_debug_time_info.find(str_vec[0]);
            if(it == m_debug_time_info.end()){    //一级目录下没有,插入一级和二级目录
                m_timetester_mutex.lock();
                std::map<std::string, Time> temp;
                temp.insert(std::pair<std::string, Time>(str_vec[1], current_time));
                m_debug_time_info.insert(std::pair<std::string, std::map<std::string, Time>>(str_vec[0], temp));
                m_timetester_mutex.unlock();
            }
            else{   //一级目录存在，插入二级目录
                auto it_t = it->second.find(str_vec[1]);
                if(it_t == it->second.end()){
                    it->second.insert(std::pair<std::string, Time>(str_vec[1], current_time));
                }
                else{ //两级目录都存在
                    it_t->second.start = now();
                }
            }
            return true;
        }
        else return false;
    }

    bool TimeTester::end(std::string label){
        if(label.empty()) return false;
        std::vector<std::string> str_vec;
        if(split(label, str_vec)){
            auto it = m_debug_time_info.find(str_vec[0]);
            if(it != m_debug_time_info.end()){ //存在一级目录
                auto it_t = it->second.find(str_vec[1]);
                if(it_t != it->second.end()){ //存在二级目录
                    it_t->second.end = now();
                }
                else return false;
            }
            else return false;
        }
        else return false;
        return true;
    }

    bool TimeTester::both(std::string label_end, std::string label_start){
        return end(label_end) && start(label_start);
    }

    bool TimeTester::both(std::string label, std::string _end, std::string _start){
        return end(label + _end) && start(label + _start);
    }

    void TimeTester::printInfo(){
        int i = 0;
        for(auto it : m_debug_time_info){
            std::cout << m_color_info[i] + it.first + ":" << std::endl;
            for(auto it_t : it.second){
                double time;
                if(it_t.second.start != 0 && it_t.second.end != 0){
                    time = (it_t.second.end - it_t.second.start);
                    if(time < 0) time = 0.;
                    if(it_t.first.find("FPS") != std::string::npos && time != 0.){
                        time = 1e3 / time;
                    }
                }
                else time = 0.;
                std::cout << m_color_info[i] + it_t.first + ": " + std::to_string(time) + "\033[0m" << std::endl;
            }
            std::cout << "\n\n";
            i++;
            if(i >= m_color_info.size()) i = 0;
        }
    }

    bool TimeTester::split(std::string str, std::vector<std::string>& str_vec){
        if(str_vec.size() < 2){
            str_vec.resize(2);
        }
        for(int i = 0; i < str.length() - 1; i++){
            if(str[i] == ' '){
                str_vec[0] = str.substr(0, i);
                str_vec[1] = str.substr(i + 1, str.length() - i - 1);
                return true;
            }
        }
        return false;
    }

}