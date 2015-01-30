#include "mbed.h"
#include <string>
#include <sstream>
#include <vector>
#include "FXOS8700CQ.h"
#include "MbedJSONValue.h"
#include "DHT.h"

#define CR              0xD
#define EXPECT          "mDot: "

#define M2X_DEVICE_ID   "abd33486151a774aa48f91cee19aef4c"
#define M2X_API_KEY     "6db8947ab6bfe46738df7b4df43adc60"

// define an alias to substitute for device id in M2X streams
#define ALIAS           "ageofsaturation"

#define ACC_X           "acc_x"
#define ACC_Y           "temp"
#define ACC_Z           "acc_z"
#define MAG_X           "mag_x"
#define MAG_Y           "mag_y"
#define MAG_Z           "mag_z"

// wait interval between sending readings
#define INTERVAL        0

Serial dot(D1, D0);
Serial pc(USBTX, USBRX);

DHT sensor(D4, DHT11);

FXOS8700CQ acc(PTE25 /* SDA */, PTE24 /* SCL */, FXOS8700CQ_SLAVE_ADDR1);

vector<string> triggers;

static bool send_command(Serial* ser, const string& tx, string& rx);
static int raw_send_command(Serial* ser, const string& tx, string& rx);
static bool rx_done(const string& rx, const string& expect);
MbedJSONValue parse_rx_messages(const string& messages);


/** Send M2X configuration data to Condit Server
 *
 */
void configureMdot()
{
    stringstream cmd;
    string res;

    printf("Configuring mDot...\r\n");

    // using stringstreams is an easy to way to get non-string data into a string
    // don't forget to reset them between uses, though
    // call .str("") and .clear() on them to reset
    
    cmd.str("");
    cmd.clear();
    cmd << "ATSEND feed-id:" << M2X_DEVICE_ID;
    printf("setting feed-id [%s]\r\n", cmd.str().c_str());
    if (! send_command(&dot, cmd.str(), res))
        printf("failed to set feed-id\r\n");

    cmd.str("");
    cmd.clear();
    cmd << "ATSEND m2x-key:" << M2X_API_KEY;
    printf("setting m2x-key [%s]\r\n", cmd.str().c_str());
    if (! send_command(&dot, cmd.str(), res))
        printf("failed to set m2x-key\r\n");
    
#ifdef ALIAS
    cmd.str("");
    cmd.clear();
    cmd << "ATSEND alias:" << ALIAS;
    printf("setting alias [%s]\r\n", cmd.str().c_str());
    if (! send_command(&dot, cmd.str(), res))
        printf("failed to set alias\r\n");
#endif

}

/** Subscribe to triggers expected to be received from M2X
 *  
 *  Triggers must be configured in M2X account before any will be received
 *
 *  See https://m2x.att.com/developer/tutorials/triggers
 */
void subscribeTriggers(const char* id)
{
    printf("subscribing to triggers\r\n");
    
    stringstream cmd;
    string res;

    // add triggers
    triggers.push_back(ACC_X);
    triggers.push_back(ACC_Y);
    triggers.push_back(ACC_Z);

    // subscribe to any triggers added
    // see the wiki page at
    // http://developer.mbed.org/teams/Multi-Hackers/wiki/ATT-Developer-Summit-Hackathon
    // for more info and context regarding triggers
    for (int i = 0; i < triggers.size(); i++) {
        cmd.str("");
        cmd.clear();
        cmd << "ATSEND subscribe:" << id << "-" << triggers[i];
        printf("subscribing [%s]\r\n", cmd.str().c_str());
        if (! send_command(&dot, cmd.str(), res))
            printf("failed to subscribe\r\n");
    }
}

/** Read device Id from mDot card
 *
 */
string getDeviceId() {
    string cmd = "ATID";
    string res;
    string id = "";
    
    // loop here till we get response
    // if we can't get the id the radio isn't working
    while (id == "") {
        if (! send_command(&dot, cmd, res)) {
            printf("failed to get device id\r\n");
        } else {
            int id_beg = res.find("Id: ");
            int id_end = res.find("\r", id_beg);
            
            if (id_beg != string::npos && id_end != string::npos) {
                id_beg += 4;                
                id = res.substr(id_beg, id_end-id_beg);
                if (id.size() == 1)
                    id = "0" + id;
            }
        }
    }
    
    return id;
}

/** Request config from Conduit server
 */
void getConfiguration()
{
    string cmd = "ATSEND config:";
    string res;
    printf("asking for config [%s]\r\n", cmd.c_str());
    if (! send_command(&dot, cmd, res))
        printf("failed to ask for config\r\n");
}

/** Request subscriptions from Conduit server
 */
void getSubscriptions()
{
    string cmd = "ATSEND subs:";
    string res;
    printf("asking for subscriptions [%s]\r\n", cmd.c_str());
    if (! send_command(&dot, cmd, res))
        printf("failed to ask for subscriptions\r\n");
}

/** Reset mDot radio
 */
void soft_radio_reset() {
    string cmd = "RESET";
    string res;
    bool done = false;
    while(!done) {
        printf("send reset radio command\r\n");
        raw_send_command(&dot, cmd, res);
        if (res.find("MultiTech Systems LoRa XBee Module") != string::npos && res.find("mDot:") != string::npos) {
            done = true;
        } else {
            printf("strings not found in reset response, [%s]\r\n", res.c_str());
        }
        wait(1);
    }
    printf("radio reset complete\r\n");
}

/** Unsubscribe from all triggers
 */
void unsubscribeAllTriggers()
{
    string cmd = "ATSEND unsubscribe:all";
    string res;
    printf("canceling all subscriptions [%s]\r\n", cmd.c_str());
    if (! send_command(&dot, cmd, res))
        printf("failed to cancel subscriptions\r\n");
}

DigitalOut led_trig(LED1);
DigitalOut led_acc_y_is_pos(LED2);
DigitalOut led_acc_y_is_neg(LED3);

int main()
{
    int error = 0; //for DHT
    float c = 0.0f;
    
    // turn off leds
    led_trig = 1;    
    led_acc_y_is_pos = 1;
    led_acc_y_is_neg = 1;
    
    dot.baud(9600);
    pc.baud(9600);

    printf("\r\n\r\nBEGIN\r\n\r\n");

    stringstream cmd;
    string res;

    // containers for acc/mag data
    SRAWDATA acc_data;
    SRAWDATA mag_data;

    // don't forget to enable the acc/mag
    acc.enable();

    string id = getDeviceId();
    printf("Device Id: %s\r\n", id.c_str());
    
#ifdef ALIAS
    id = ALIAS;
    printf("Device Alias: %s\r\n", id.c_str());
    unsubscribeAllTriggers();
#endif

    // mDot should only need to be configured once
    // ask server for config first and see if it matches
    getConfiguration();
    getSubscriptions();

    printf("waiting for server to respond...\r\n");
    wait(5);

    
    Timer report_timer;

    while (true) {
        report_timer.start();
        
        printf("\r\nchecking messages\r\n");
        if (send_command(&dot, "ATRECV", res)) {
            MbedJSONValue messages = parse_rx_messages(res);

            if (messages.size() > 0) {
                printf("\r\nparsed %d json messages\r\n\r\n", messages.size());
            } else {
                printf("\r\nNo messages received.\r\n");
            }

            // check messages for config, subscriptions and triggers
            for (int i = 0; i < messages.size(); i++) {
                printf("message %d\r\n", i);
                if (messages[i].hasMember("config")) {
                    // display configuration
                    printf("config %s\r\n", messages[i]["config"].serialize().c_str());

                    bool configured = false;

                    MbedJSONValue &config = messages[i]["config"];

                    if (config.hasMember("feed-id") && config.hasMember("m2x-key")) {
                        // verify configuration
                        string dev_id = config["feed-id"].get<string>();
                        string api_key = config["m2x-key"].get<string>();
                        
                        #ifdef ALIAS                        
                        string alias = id;                                                
                        if (!(config.hasMember("alias") && alias.compare(ALIAS) == 0 && alias.compare(config["alias"].get<string>()) == 0)) {
                            printf("mDot alias did not match\r\n");
                        } else 
                        #endif
                        
                        if (dev_id.compare(M2X_DEVICE_ID) != 0) {
                            printf("mDot m2x feed-id did not match\r\n");
                            printf("configured: '%s' expected: '%s'\r\n", dev_id.c_str(), M2X_DEVICE_ID);
                        } else if (api_key.compare(M2X_API_KEY) != 0) {
                            printf("mDot m2x api-key did not match\r\n");
                            printf("configured: '%s' expected: '%s'\r\n", api_key.c_str(), M2X_API_KEY);                            
                        } else {
                            printf("mDot configured correctly\r\n");
                            configured = true;
                        }
                    }

                    if (!configured) {
                        configureMdot();
                        getConfiguration();
                        printf("waiting for server to respond...\r\n");
                        wait(5);
                        continue;
                    }

                } else if (messages[i].hasMember("subs")) {
                    // display subscriptions
                    MbedJSONValue& subs = messages[i]["subs"];
                    
                    if (subs.size() > 0) {
                        printf("%d subscriptions\r\n", subs.size());
                        for (int j = 0; j < subs.size(); j++) {
                            printf("%d - %s\r\n", j, subs[j].get<string>().c_str());
                        }
                    } else {
                        subscribeTriggers(id.c_str());
                        getSubscriptions();
                        printf("waiting for server to respond...\r\n");
                        wait(5);
                        continue;
                    }
                } else if (messages[i].hasMember("s") && messages[i].hasMember("v")) {
                    // blink rssi led
                    led_trig = 0;
                    wait(.1);
                    led_trig = 1;
                    wait(.1);
                    led_trig = 0;
                    wait(.1);
                    led_trig = 1;
                    
                    string stream = messages[i]["s"].get<string>();
                    
                    if (stream.find("acc_y") != string::npos) {
                        // turn off led
                        led_acc_y_is_neg = led_acc_y_is_pos = 1;
                        
                        // check type and change led color
                        // Blue if acc_y is negative
                        // Green if acc_y is positive
                        if (messages[i]["v"].getType() == MbedJSONValue::TypeInt) {                            
                            if (messages[i]["v"].get<int>() > 0) {
                                led_acc_y_is_pos = 0;                                
                            } else {
                                led_acc_y_is_neg = 0;
                            }
                        } else if (messages[i]["v"].getType() == MbedJSONValue::TypeDouble) {
                            if (messages[i]["v"].get<double>() > 0) {
                                led_acc_y_is_pos = 0;                                
                            } else {
                                led_acc_y_is_neg = 0;
                            }
                        }
                    }
                    
                    // display trigger response
                    printf("trigger-stream: '%s'\r\n", stream.c_str());
                    
                    if (messages[i]["v"].getType() == MbedJSONValue::TypeInt)
                        printf("trigger-value: %d\r\n", messages[i]["v"].get<int>());
                    else if (messages[i]["v"].getType() == MbedJSONValue::TypeDouble)
                        printf("trigger-value: %f\r\n", messages[i]["v"].get<double>());
                    else
                        printf("trigger-value: '%s'\r\n\r\n", messages[i]["v"].get<string>().c_str());
                }

                printf("\r\n");
            }

        } else {
            printf("failed to check messages\r\n");
        }


        if (report_timer.read() > INTERVAL) {
            printf("\r\nCurrent sensor readings\r\n");
            acc.get_data(&acc_data, &mag_data);
            printf("acc: x %d, y %d, z %d\r\n", acc_data.x, acc_data.y, acc_data.z);
            
            error = sensor.readData();
                if (0 == error) {
                    c   = sensor.ReadTemperature(CELCIUS);
                    }
                else
                    c = 0;   
            printf("%f \r\n", c);
            //printf("mag: x %d, y %d, z %d\r\n\r\n", mag_data.x, mag_data.y, mag_data.z);
    
            int call_check = 0;
        
            cmd.str("");
            cmd.clear();
            cmd << "ATSEND " << ACC_X << ":" << acc_data.x;
            printf("sending %s [%s]\r\n", ACC_X, cmd.str().c_str());
            if (! send_command(&dot, cmd.str(), res)) {
                printf("failed to send %s\r\n", ACC_X);
                call_check++;
            }
            cmd.str("");
            cmd.clear();
            
            
            //cmd << "ATSEND " << ACC_Y << ":" << acc_data.y;
            cmd << "ATSEND " << ACC_Y << ":" << c;
            printf("sending %s [%s]\r\n", ACC_Y, cmd.str().c_str());
            if (! send_command(&dot, cmd.str(), res)) {
                printf("failed to send %s\r\n", ACC_Y);
                call_check++;
            }
                
            /*    
            cmd.str("");
            cmd.clear();        
            cmd << "ATSEND " << ACC_Z << ":" << acc_data.z;
            printf("sending %s [%s]\r\n", ACC_Z, cmd.str().c_str());
            if (! send_command(&dot, cmd.str(), res)) {
                printf("failed to send %s\r\n", ACC_Y);
                call_check++;
            }
            
            cmd.str("");
            cmd.clear();
            cmd << "ATSEND " << MAG_X << ":" << mag_data.x;
            printf("sending %s [%s]\r\n", MAG_X, cmd.str().c_str());
            if (! send_command(&dot, cmd.str(), res)) {
                printf("failed to send %s\r\n", ACC_Y);
                call_check++;
            }
                
            cmd.str("");
            cmd.clear();
            cmd << "ATSEND " << MAG_Y << ":" << mag_data.y;
            printf("sending %s [%s]\r\n", MAG_Y, cmd.str().c_str());
            if (! send_command(&dot, cmd.str(), res)) {
                printf("failed to send %s\r\n", ACC_Y);
                call_check++;
            }
                
            cmd.str("");
            cmd.clear();
            cmd << "ATSEND " << MAG_Z << ":" << mag_data.z;
            printf("sending %s [%s]\r\n", MAG_Z, cmd.str().c_str());
            if (! send_command(&dot, cmd.str(), res)) {
                printf("failed to send %s\r\n", ACC_Y);
                call_check++;
            }*/
    
            if (call_check > 1) {
                printf("failed calls exceeded 1/6\r\n");
                soft_radio_reset();
            }
            report_timer.reset();
        }
        
        //wait(1);
    }

    return 0;
}

/** Parse ATRECV output for json messages
 */
MbedJSONValue parse_rx_messages(const string& messages)
{
    MbedJSONValue msgs;

    // find start of first json object
    int beg_pos = messages.find("{");
    int end_pos = 0;
    int count = 0;

    while (beg_pos != string::npos) {
        // find end of line
        end_pos = messages.find("\r", beg_pos);

        // pull json string from messages
        string json = messages.substr(beg_pos, end_pos-beg_pos);

        // parse json string
        string ret = parse(msgs[count], json.c_str());

        if (ret != "") {
            printf("invalid json: '%s'\r\n", json.c_str());
        } else {
            count++;
        }

        // find start of next json object
        beg_pos = messages.find("{", end_pos);
    }

    return msgs;
}

/** Send a command to the mDot radio expecting an OK response
 *
 */
static bool send_command(Serial* ser, const string& tx, string& rx)
{
    int ret;

    rx.clear();

    if ((ret = raw_send_command(ser, tx, rx)) < 0) {
        printf("raw_send_command failed %d\r\n", ret);
        return false;
    }
    if (rx.find("OK") == string::npos) {
        printf("no OK in response\r\n");
        return false;
    }

    return true;
}

/** Check response for expected string value
 */
static bool rx_done(const string& rx, const string& expect)
{
    return (rx.find(expect) == string::npos) ? false : true;
}

/** Send command to mDot radio without checking response
 */
static int raw_send_command(Serial* ser, const string& tx, string& rx)
{
    if (! ser)
        return -1;

    int to_send = tx.size();
    int sent = 0;
    Timer tmr;
    string junk;

    // send a CR and read any leftover/garbage data
    ser->putc(CR);
    tmr.start();
    while (tmr.read_ms() < 500 && !rx_done(junk, EXPECT))
        if (ser->readable())
            junk += ser->getc();

    tmr.reset();
    tmr.start();
    while (sent < to_send && tmr.read_ms() <= 1000)
        if (ser->writeable())
            ser->putc(tx[sent++]);

    if (sent < to_send)
        return -1;

    // send newline after command
    ser->putc(CR);

    tmr.reset();
    tmr.start();
    while (tmr.read_ms() < 10000 && !rx_done(rx, EXPECT))
        if (ser->readable())
            rx += ser->getc();

    return rx.size();
}
