#ifndef UGV_CAN_BRIDGE_H_
#define UGV_CAN_BRIDGE_H_

#include "controlcan.h"
#include <pthread.h>
#include <vector>
#include "can_msgs/Frame.h"
#include <mutex>

using std::vector;
using std::mutex;
using std::unique_lock;

namespace cantools {
    class CanBridge {
    public:
        CanBridge();
        ~CanBridge();
        bool Write(const vector<can_msgs::Frame>& msgs);
//        bool WriteLight(const vector<can_msgs::Frame>& msgs);

        int Read(vector<can_msgs::Frame> &frames);

        void CanClose();
        void CanOpen();

    private:
        VCI_BOARD_INFO board_info_;
        static void* ReceiveFunc(void* param);

        pthread_t thread_0_;
        static void* FuncHelper(void* a){
            return (void*) ReceiveFunc(&run_flag_);
        }
        static unsigned int device_index_;
        static unsigned int can_index_;
  //      static unsigned int light_index_;
        static int run_flag_;
        static vector<can_msgs::Frame> can_buf_;
        static can_msgs::Frame can_tmp_;
        static mutex can_mutex_sync_;
    };
}
#endif
