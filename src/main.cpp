#include "main.h"
JOINT_INFO info;
double gt = 0;
void set_thread_affinity(pthread_t thread, int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    int result = pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset);
    if (result != 0) {
        printf("Error setting thread affinity: %d\n", result);
    }
}
void signal_handler(int signum) {
	printf("\n\n");
	is_run  =0;
	std::vector<HJVec> tau_list;
    HJVec tau1,tau2,tau3,tau4=HJVec::Zero();
    tau_list.push_back(tau1);
    tau_list.push_back(tau2);
    tau_list.push_back(tau3);
    tau_list.push_back(tau4);
    delto.set_tau(tau_list);
	usleep(20000);
	delto.set_tau(tau_list);
	usleep(20000);
	delto.set_tau(tau_list);
	usleep(20000);
    std::cout<<"END"<<std::endl;
	if(signum==SIGINT)
		printf("╔════════════════[SIGNAL INPUT SIGINT]═══════════════╗\n");
	else if(signum==SIGTERM)
		printf("╔═══════════════[SIGNAL INPUT SIGTERM]═══════════════╗\n");	
	else if(signum==SIGWINCH)
		printf("╔═══════════════[SIGNAL INPUT SIGWINCH]══════════════╗\n");		
	else if(signum==SIGHUP)
		printf("╔════════════════[SIGNAL INPUT SIGHUP]═══════════════╗\n");
        printf("║                Servo drives Stopped!               ║\n");
        printf("╚════════════════════════════════════════════════════╝\n");		
    exit(1);
}
int is_run =1;
Hand delto;


int main(void) {
	const std::string hostname = "169.254.186.72"; //STEP2 IP Address
	const Poco::UInt16 PORT = 10000;
	 delto =  Hand(hostname,PORT);

    pthread_t qt_thread, control_thread;
    pthread_attr_t qt_attr, control_attr;
    struct sched_param qt_param, control_param;

    // qt_thread 설정 및 생성
    pthread_attr_init(&qt_attr);
    pthread_attr_setschedpolicy(&qt_attr, SCHED_RR);
    qt_param.sched_priority = sched_get_priority_max(SCHED_RR) - 50;
    pthread_attr_setschedparam(&qt_attr, &qt_param);
    pthread_create(&qt_thread, &qt_attr, qt_run, NULL);
    set_thread_affinity(qt_thread, 2); // 2번 코어에 할당

    // control_thread 설정 및 생성
    pthread_attr_init(&control_attr);
    pthread_attr_setschedpolicy(&control_attr, SCHED_RR);
    control_param.sched_priority = sched_get_priority_max(SCHED_RR) - 40; // qt_thread보다 높은 우선순위 설정
    pthread_attr_setschedparam(&control_attr, &control_param);
    pthread_create(&control_thread, &control_attr, control_run, NULL);

    // 신호 설정 및 대기
    sigset_t set;
    int sig, ret;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGHUP);
    pthread_sigmask(SIG_BLOCK, &set, NULL);

    ret = sigwait(&set, &sig);
    signal_handler(sig);

    // 스레드 종료 대기
    pthread_join(qt_thread, NULL);
    pthread_join(control_thread, NULL);

    return 0;
}