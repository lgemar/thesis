#include "mex.h"
#include "class_handle.hpp"
#include <lcm/lcm-cpp.hpp>
#include "body_t.hpp"
#include <string>
#include <thread>

using namespace std;
using namespace lcm;
using namespace vicon;

// The class that we are interfacing to
class ViconLCM {
private:
    double m_t;
    double m_x[7];
    LCM m_lcm;
    thread m_thread;
    mutex m_mutex;
    void handleMessage(const ReceiveBuffer* rbuf, const string& chan, const body_t* msg) {
        m_mutex.lock();
        m_t = double(msg->utime)/1000.0;
        m_x[0] = msg->trans[0];
        m_x[1] = msg->trans[1];
        m_x[2] = msg->trans[2];
        m_x[3] = msg->quat[0];
        m_x[4] = msg->quat[1];
        m_x[5] = msg->quat[2];
        m_x[6] = msg->quat[3];
        m_mutex.unlock();
    };
    void listen() {
        while(1) {
            if(m_lcm.handle() !=0) {
                mexErrMsgTxt("Something went wrong in lcm.handle.");
            }
        }
    };
public:
    void subscribe(string chan) {
        m_lcm.subscribe(chan, &ViconLCM::handleMessage, this);
        m_thread = thread(&ViconLCM::listen, this); //kick off listener thread
    };
    double getNextMessage(double *x, int timeout_millis) {
        m_lcm.handleTimeout(timeout_millis);
        return getCurrentValue(x);
    };
    double getCurrentValue(double *x) {
        m_mutex.lock();
        for(int k = 0; k < 7; ++k) {
            x[k] = m_x[k];
        }
        double t = m_t;
        m_mutex.unlock();
        return t;
    };
    double getLastTimestamp() {
        m_mutex.lock();
        double t = m_t;
        m_mutex.unlock();
        return t;
    };
};

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {	
    
    char cmd[64];
        
    //New
    if(nrhs == 1) {
        mxGetString(prhs[0], cmd, sizeof(cmd));
        if(strcmp("new", cmd) == 0 && nlhs == 1) {
            // Return a handle to a new C++ instance
            plhs[0] = convertPtr2Mat<ViconLCM>(new ViconLCM());
            return;
        }
        else {
            mexErrMsgTxt("Incorrect number of inputs and/or outputs.");
            return;
        }
    }
    
    //Other function calls
    if(nrhs > 1) {
        //First argument should be a pointer to an instance of ViconLCM
        ViconLCM *vicon_lcm = convertMat2Ptr<ViconLCM>(prhs[0]);
        
        //Second argument should be the function name
        mxGetString(prhs[1], cmd, sizeof(cmd));
        if(strcmp("getCurrentValue", cmd) == 0) {
            plhs[0] = mxCreateNumericMatrix(7, 1, mxDOUBLE_CLASS, mxREAL);
            double *p_x = mxGetPr(plhs[0]);
            plhs[1] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
            double *p_t = mxGetPr(plhs[1]);
            p_t[0] = vicon_lcm->getCurrentValue(p_x);
        }
        else if(strcmp("getLastTimestamp", cmd) == 0) {
            plhs[0] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
            double *p_t = mxGetPr(plhs[0]);
            p_t[0] = vicon_lcm->getLastTimestamp();
        }
        else if(strcmp("getNextMessage", cmd) == 0) {
            //Third argument should be the timeout in ms
            double *p_timeout = mxGetPr(prhs[2]);
            int timeout_millis = int(p_timeout[0]);
            
            plhs[0] = mxCreateNumericMatrix(7, 1, mxDOUBLE_CLASS, mxREAL);
            double *p_x = mxGetPr(plhs[0]);
            plhs[1] = mxCreateNumericMatrix(1, 1, mxDOUBLE_CLASS, mxREAL);
            double *p_t = mxGetPr(plhs[1]);
            p_t[0] = vicon_lcm->getNextMessage(p_x, timeout_millis);
        }
        else if(strcmp("subscribe", cmd) == 0) {
            //Third argument should be the channel name
            char chan[64];
            mxGetString(prhs[2], chan, sizeof(chan));
            
            vicon_lcm->subscribe(std::string(chan));
            return;
        }
        else if(strcmp("delete", cmd) == 0) {
            //Destroy the C++ object
            destroyObject<ViconLCM>(prhs[0]);
            return;
        }
        else {
            mexErrMsgTxt("Invalid function name.");
            return;
        }
    }
    
    else {
        mexErrMsgTxt("Not enough input arguments.");
        return;
    }
}
