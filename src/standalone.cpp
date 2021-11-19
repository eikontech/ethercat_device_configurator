/*
 ** Copyright 2021 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
** Simple Example executable for the use of the RSL EtherCAT software tools
** ════════════════════════════════════════════════════════════════════════
**
**   To understand the logic it is best to start at the main function and
**   then go through the code step by step. The executable compiled from
**   this source code needs to be executed as root. Simply executing the
**   executable with sudo will not work because the catkin workspace won’t
**   be sourced and the linker cannot find the necessary libraries. Use the
**   following command to start this executable:
**   ┌────
**   │ sudo bash -c 'source /absolute/path/to/your/catkin_ws/devel/setup.bash; path/to/executable path/to/setup.yaml'
**   └────
**
**
** Build errors
** ────────────
**
**   Are you using gcc and g++ both with a version >= 8.4? See the
**   README.md for more details.
 */
#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"

#ifdef _ANYDRIVE_FOUND_
#include <anydrive/Anydrive.hpp>
#endif
#ifdef _ELMO_FOUND_
#include <elmo_ethercat_sdk/Elmo.hpp>
#endif
#ifdef _OPUS_FOUND_
#include <opus_ethercat_sdk/Opus.hpp>
#endif
#ifdef _MAXON_FOUND_
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#endif
#ifdef _ROKUBI_FOUND_
#include <rokubimini_rsl_ethercat/RokubiminiEthercat.hpp>
#endif
#include <thread>
#include <csignal>
#include <pthread.h>

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 500

pthread_t worker_thread1;
std::unique_ptr<std::thread> worker_thread;
std::unique_ptr<std::thread> ecatcheck_thread;
bool abrt = false;

EthercatDeviceConfigurator::SharedPtr configurator;

unsigned int counter = 0;

struct sched_param schedp;
struct timeval tv, t1, t2;
int dorun = 0;
int deltat, tmax = 0;
int64 toff, gl_delta;
int DCdiff;
int os;
uint8 ob;
uint16 ob2;
uint8 *digout = 0;
int expectedWKC = 3;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

int operation_mode = 8;

/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
    int64 sec, nsec;

    nsec = addtime % NSEC_PER_SEC;
    sec = (addtime - nsec) / NSEC_PER_SEC;
    ts->tv_sec += sec;
    ts->tv_nsec += nsec;
    if (ts->tv_nsec > NSEC_PER_SEC)
    {
        nsec = ts->tv_nsec % NSEC_PER_SEC;
        ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
        ts->tv_nsec = nsec;
    }
}

/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime, int64 *offsettime)
{
    static int64 integral = 0;
    int64 delta;
    /* set linux sync point 50us later than DC sync, just as example */
    delta = (reftime - 50000) % cycletime;
    if (delta > (cycletime / 2))
    {
        delta = delta - cycletime;
    }
    if (delta > 0)
    {
        integral++;
    }
    if (delta < 0)
    {
        integral--;
    }
    *offsettime = -(delta / 100) - (integral / 20);
    gl_delta = delta;
}

void *worker2(void *ptr)
{
    struct timespec ts, tleft;
    int ht;
    int64 cycletime;

    // uint32 buf32;
    // // uint16 buf16;
    // uint8 buf8;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
    ts.tv_nsec = ht * 1000000;
    cycletime = *(int *)ptr * 1000; /* cycletime in ns */
    toff = 0;
    dorun = 0;
    uint counter = 0;
    // int blink = 0;

    ec_send_processdata();
    while (!abrt)
    {
        counter++;
        /* calculate next cycle start */
        add_timespec(&ts, cycletime + toff);
        /* wait to cycle start */
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
        if (dorun > 0)
        {
            /*
            ** Update each master.
            ** This sends the last staged commands and reads the latest readings over EtherCAT.
            ** The StandaloneEnforceRate update mode is used.
            ** This means that average update rate will be close to the target rate (if possible).
            */
            for (const auto &master : configurator->getMasters())
            {
                master->update(ecat_master::UpdateMode::NonStandalone); // TODO fix the rate compensation (Opus reliability problem)!!
            }

            dorun++;

            if (ec_slave[0].hasdc)
            {
                /* calulate toff to get linux time and DC synced */
                ec_sync(ec_DCtime, cycletime, &toff);
            }
        }
    }
    return NULL;
}

void worker()
{
    bool rtSuccess = true;
    for (const auto &master : configurator->getMasters())
    {
        rtSuccess &= master->setRealtimePriority(99, 1);
    }
    std::cout << "Setting RT Priority (attached to core 1): " << (rtSuccess ? "successful." : "not successful. Check user privileges.") << std::endl;

    bool blink = false;
    double error_cumulative = 0;
    double kp = 0.5;
    double ki = 0.01;

    // Flag to set the drive state for the elmos on first startup
#ifdef _ELMO_FOUND_
    bool elmoEnabledAfterStartup = false;
#endif
    // Flag to set the drive state for the opuss on first startup
#ifdef _OPUS_FOUND_
    bool opusEnabledAfterStartup = false;
    bool l_driveStateRequestSent = false;
#endif
    // Flag to set the drive state for the maxons on first startup
#ifdef _MAXON_FOUND_
    bool maxonEnabledAfterStartup = false;
#endif
    /*
    ** The communication update loop.
    ** This loop is supposed to be executed at a constant rate.
    ** The EthercatMaster::update function incorporates a mechanism
    ** to create a constant rate.
     */
    while (!abrt)
    {
        /*
        ** Update each master.
        ** This sends the last staged commands and reads the latest readings over EtherCAT.
        ** The StandaloneEnforceRate update mode is used.
        ** This means that average update rate will be close to the target rate (if possible).
         */
        for (const auto &master : configurator->getMasters())
        {
            master->update(ecat_master::UpdateMode::StandaloneEnforceStep);
        }

        /*
        ** Do things with the attached devices.
        ** Your lowlevel control input / measurement logic goes here.
        ** Different logic can be implemented for each device.
         */
        for (const auto &slave : configurator->getSlaves())
        {
            // Anydrive
            if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Anydrive)
            {
#ifdef _ANYDRIVE_FOUND_
                anydrive::AnydriveEthercatSlave::SharedPtr any_slave_ptr = std::dynamic_pointer_cast<anydrive::AnydriveEthercatSlave>(slave);

                if (any_slave_ptr->getActiveStateEnum() == anydrive::fsm::StateEnum::ControlOp)
                {
                    anydrive::Command cmd;
                    cmd.setModeEnum(anydrive::mode::ModeEnum::MotorVelocity);
                    cmd.setMotorVelocity(10);

                    any_slave_ptr->setCommand(cmd);
                }
#endif
            }
            // Rokubi
            else if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Rokubi)
            {
#ifdef _ROKUBI_FOUND_
                std::shared_ptr<rokubimini::ethercat::RokubiminiEthercat> rokubi_slave_ptr = std::dynamic_pointer_cast<rokubimini::ethercat::RokubiminiEthercat>(slave);
                // Do things with the Rokubi sensors here
#endif
            }

            // Elmo
            else if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Elmo)
            {
#ifdef _ELMO_FOUND_
                std::shared_ptr<elmo::Elmo> elmo_slave_ptr = std::dynamic_pointer_cast<elmo::Elmo>(slave);
                if (!elmoEnabledAfterStartup)
                    // Set elmos to operation enabled state, do not block the call!
                    elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);
                // set commands if we can
                if (elmo_slave_ptr->lastPdoStateChangeSuccessful() && elmo_slave_ptr->getReading().getDriveState() == elmo::DriveState::OperationEnabled)
                {
                    elmo::Command command;
                    command.setTargetVelocity(50);
                    elmo_slave_ptr->stageCommand(command);
                }
                else
                {
                    MELO_WARN_STREAM(rclcpp::get_logger("standalone"), "Elmo '" << elmo_slave_ptr->getName() << "': " << elmo_slave_ptr->getReading().getDriveState());
                    //elmo_slave_ptr->setDriveStateViaPdo(elmo::DriveState::OperationEnabled, false);
                }
                auto reading = elmo_slave_ptr->getReading();
                // std::cout << "Elmo '" << elmo_slave_ptr->getName() << "': "
                //                 << "velocity: " << reading.getActualVelocity() << " rad/s\n";
#endif
            }
            // Opus
            else if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Opus)
            {
                MELO_INFO_STREAM(rclcpp::get_logger("standalone"), "Slave OPUS found!\n");
                // #ifdef _OPUS_FOUND_
                std::shared_ptr<opus::Opus> slave_ptr = std::dynamic_pointer_cast<opus::Opus>(slave);

                if (!opusEnabledAfterStartup)
                {
                    slave_ptr->setDriveStateViaPdo(opus::DriveState::OperationEnabled, false);
                    l_driveStateRequestSent = true;
                }

                // if (slave_ptr->getReading().hasUnreadError())
                {
                    auto l_last = slave_ptr->getReading().getLastError();
                    MELO_INFO_STREAM(rclcpp::get_logger("standalone"), "Opus '" << slave_ptr->getName() << "' error: " << static_cast<int>(l_last));
                }

                // if (slave_ptr->getReading().hasUnreadFault())
                {
                    auto l_last = slave_ptr->getReading().getLastFault();
                    MELO_INFO_STREAM(rclcpp::get_logger("standalone"), "Opus '" << slave_ptr->getName() << "' fault: " << static_cast<int>(l_last));
                }

                if (counter % 100000 == 0)
                    blink = !blink;
                double factor = blink ? -1. : 1.;
                double velocity_rads = 0.05;
                double max_velocity_rads = 0.5;
                double desired_position_rad = 6.28 * factor;

                if (1)
                {
                    double error = desired_position_rad - slave_ptr->getReading().getActualPosition();
                    if (fabs(error) < 0.01)
                    {
                        error_cumulative = 0;
                        velocity_rads = 0;
                    }
                    else
                    {
                        velocity_rads = kp * error + ki * error_cumulative;
                        error_cumulative += error;
                        if (fabs(velocity_rads) > max_velocity_rads)
                            velocity_rads = velocity_rads > 0 ? max_velocity_rads : -max_velocity_rads;
                    }
                }

                // set commands if we can
                if (slave_ptr->lastPdoStateChangeSuccessful() && slave_ptr->getReading().getDriveState() == opus::DriveState::OperationEnabled)
                {
                    opus::Command command;
                    if (0)
                    {
                        command.setTargetVelocity(velocity_rads);
                        MELO_INFO_STREAM(rclcpp::get_logger("standalone"), "Set target Velocity:\n"
                                                                               << command);
                    }
                    else
                    {
                        command.setModeOfOperation(opus::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                        auto reading = slave_ptr->getReading();
                        command.setTargetPosition(reading.getActualPosition() + 10);
                        command.setTargetTorque(factor*1.5);
                        MELO_INFO_STREAM(rclcpp::get_logger("standalone"), "Set target Torque:\n"
                                                                               << command);
                    }
                    slave_ptr->stageCommand(command);

                    // MELO_INFO_STREAM(rclcpp::get_logger("standalone"), "Opus '" << slave_ptr->getName() << "': " << slave_ptr->getReading().getDriveState());
                    // l_driveStateRequestSent = false;
                }
                else
                {
                    if (slave_ptr->getReading().getDriveState() == opus::DriveState::Fault)
                        l_driveStateRequestSent = false;

                    MELO_WARN_STREAM(rclcpp::get_logger("standalone"), "Opus '" << slave_ptr->getName() << "': " << slave_ptr->getReading().getDriveState());

                    if (!l_driveStateRequestSent)
                    {
                        l_driveStateRequestSent = true;
                        // Set opus to operation enabled state, do not block the call!
                        slave_ptr->setDriveStateViaPdo(opus::DriveState::OperationEnabled, false);
                    }
                }

                std::cout << "Opus '" << slave_ptr->getName() << "': "
                          << "actual  position: " << slave_ptr->getReading().getActualPosition() << " rad, "
                          << "actual  velocity: " << slave_ptr->getReading().getActualVelocity() << " rad/s, "
                          << "torque: " << slave_ptr->getReading().getActualTorque() << " N/m\n"
                          << "desired position: " << desired_position_rad << " rad, "
                          << "desired velocity: " << velocity_rads << " rad/s\n";

                std::cout << "Opus '" << slave_ptr->getName() << "': "
                          << "position RAW: " << slave_ptr->getReading().getActualPositionRaw() << " cnt\n";
                // #endif
            }

            // Maxon
            else if (configurator->getInfoForSlave(slave).type == EthercatDeviceConfigurator::EthercatSlaveType::Maxon)
            {
#ifdef _MAXON_FOUND_

                // Keep constant update rate
                // auto start_time = std::chrono::steady_clock::now();

                std::shared_ptr<maxon::Maxon> maxon_slave_ptr = std::dynamic_pointer_cast<maxon::Maxon>(slave);

                if (!maxonEnabledAfterStartup)
                {
                    // Set maxons to operation enabled state, do not block the call!
                    maxon_slave_ptr->setDriveStateViaPdo(maxon::DriveState::OperationEnabled, false);
                }

                // set commands if we can
                if (maxon_slave_ptr->lastPdoStateChangeSuccessful() &&
                    maxon_slave_ptr->getReading().getDriveState() == maxon::DriveState::OperationEnabled)
                {
                    maxon::Command command;
                    command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode);
                    auto reading = maxon_slave_ptr->getReading();
                    command.setTargetPosition(reading.getActualPosition() + 10);
                    command.setTargetTorque(-0.5);
                    maxon_slave_ptr->stageCommand(command);
                }
                else
                {
                    MELO_WARN_STREAM(rclcpp::get_logger("standalone"), "Maxon '" << maxon_slave_ptr->getName()
                                                                                 << "': " << maxon_slave_ptr->getReading().getDriveState());
                }

                // Constant update rate
                // std::this_thread::sleep_until(start_time + std::chrono::milliseconds(1));

#endif
            }
        }
        counter++;
#ifdef _ELMO_FOUND_
        elmoEnabledAfterStartup = true;
#endif
#ifdef _OPUS_FOUND_
        opusEnabledAfterStartup = true;
#endif
#ifdef _MAXON_FOUND_
        maxonEnabledAfterStartup = true;
#endif
    }
}

void ecatcheck()
{
    while (!abrt)
    {
        for (const auto &master : configurator->getMasters())
        {
            master->ecatCheck();
        }
        osal_usleep(10000);
    }
}

/*
** Handle the interrupt signal.
** This is the shutdown routine.
** Note: This logic is executed in a thread separated from the communication update!
 */
void signal_handler(int sig)
{
    /*
    ** Pre shutdown procedure.
    ** The devices execute procedures (e.g. state changes) that are necessary for a
    ** proper shutdown and that must be done with PDO communication.
    ** The communication update loop (i.e. PDO loop) continues to run!
    ** You might thus want to implement some logic that stages zero torque / velocity commands
    ** or simliar safety measures at this point using e.g. atomic variables and checking them
    ** in the communication update loop.
     */
    for (const auto &master : configurator->getMasters())
    {
        master->preShutdown();
    }

    // stop the PDO communication at the next update of the communication loop
    abrt = true;
    worker_thread->join();

    /*
    ** Completely halt the EtherCAT communication.
    ** No online communication is possible afterwards, including SDOs.
     */
    for (const auto &master : configurator->getMasters())
    {
        master->shutdown();
    }

    // Exit this executable
    std::cout << "Shutdown" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    exit(0);
}

#ifdef _ANYDRIVE_FOUND_
// Some dummy callbacks
void anydriveReadingCb(const std::string &name, const anydrive::ReadingExtended &reading)
{
    // std::cout << "Reading of anydrive '" << name << "'\n"
    //           << "Joint velocity: " << reading.getState().getJointVelocity() << "\n\n";
}
#endif
#ifdef _ROKUBI_FOUND_
void rokubiReadingCb(const std::string &name, const rokubimini::Reading &reading)
{
    // std::cout << "Reading of rokubi '" << name << "'\n"
    //           << "Force X: " << reading.getForceX() << "\n\n";
}
#endif

#define stack64k (64 * 1024)

/*
** Program entry.
** Pass the path to the setup.yaml file as first command line argument.
 */
int main(int argc, char **argv)
{
    // Set the abrt_ flag upon receiving an interrupt signal (e.g. Ctrl-c)
    std::signal(SIGINT, signal_handler);

    if (argc < 2)
    {
        std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
        return EXIT_FAILURE;
    }

    /* create RT thread */
    // osal_thread_create_rt(&thread1, stack64k * 2, &ecatthread, (void *)&ctime_us);

    // a new EthercatDeviceConfigurator object (path to setup.yaml as constructor argument)
    configurator = std::make_shared<EthercatDeviceConfigurator>(argv[1]);

    /*
    ** Add callbacks to the devices that support them.
    ** If you don't want to use callbacks this part can simply be left out.
    ** configurator->getSlavesOfType is another way of extracting only the devices
    ** of a ceratin type.
     */
#ifdef _ANYDRIVE_FOUND_
    for (const auto &device : configurator->getSlavesOfType<anydrive::AnydriveEthercatSlave>())
    {
        device->addReadingCb(anydriveReadingCb);
    }
#endif
#if _ROKUBI_FOUND_
    for (const auto &device : configurator->getSlavesOfType<rokubimini::ethercat::RokubiminiEthercat>())
    {
        device->addReadingCb(rokubiReadingCb);
    }
#endif

    // int ctime = 250; // micro seconds
    // osal_thread_create_rt(&worker_thread1, stack64k * 2, (void *)&worker2, (void *)&ctime);

    dorun = 1;
    /*
    ** Start all masters.
    ** There is exactly one bus per master which is also started.
    ** All online (i.e. SDO) configuration is done during this call.
    ** The EtherCAT interface is active afterwards, all drives are in Operational
    ** EtherCAT state and PDO communication may begin.
     */
    for (auto &master : configurator->getMasters())
    {
        if (!master->startup())
        {
            std::cerr << "Startup not successful." << std::endl;
            return EXIT_FAILURE;
        }
    }

    MELO_DEBUG_STREAM(m_logger, "[" << __FUNCTION__ << "] Run Worker Thread!");
    // Start the PDO loop in a new thread.
    worker_thread = std::make_unique<std::thread>(&worker);
    ecatcheck_thread = std::make_unique<std::thread>(&ecatcheck);

    std::cout << "Startup finished" << std::endl;
    // pthread_join(worker_thread1, NULL);

    std::cout << "waiting for worker thread to finish..." << std::endl;
    worker_thread->join();
    // nothing further to do in this thread.
    pause();
}
