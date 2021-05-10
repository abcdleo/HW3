#include "mbed.h"
#include "mbed_rpc.h"
#include <cmath>
#include "stm32l475e_iot01_accelero.h"

/////**********************************/////
#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
/////**********************************/////

void getAcc(Arguments *in, Reply *out);

BufferedSerial pc(USBTX, USBRX);
void gesture_UI(Arguments *in, Reply *out);
void tilt_angle(Arguments *in, Reply *out);
void detect_angle();
int detect_gesture();
inline double length(int16_t * DataXYZ);
inline double square(double x);
void start_tilt();
RPCFunction rpcUI(&gesture_UI, "gesture_UI");  // /gesture_UI/run
RPCFunction rpcangle(&tilt_angle, "tilt_angle");  // /tilt_angle/run

/////**********************************/////
int PredictGesture(float* output);
/////**********************************/////

InterruptIn button(USER_BUTTON);
DigitalIn sel(D10); //sel
DigitalOut UIled(LED1);
DigitalOut tiltled(LED2);
DigitalOut initled(LED3);

Thread thread_tilt;
Thread thread_UI;

/**********************************/
WiFiInterface *wifi;

volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Mbed";

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;
void messageArrived(MQTT::MessageData& md);
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
void close_mqtt();
/**********************************/

int angle_threshold = 30;
bool confirm = false;
bool gesture_key = false;
bool angle_key = false;

/////**********************************/////
// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
/////**********************************/////

int main() {

	BSP_ACCELERO_Init();
	char buf[256], outbuf[256];

   	FILE *devin = fdopen(&pc, "r");
   	FILE *devout = fdopen(&pc, "w");

	/***************************/
  	wifi = WiFiInterface::get_default_instance();
      	if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
      	}

    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
        printf("\nConnection error: %d\r\n", ret);
        return -1;
    }

    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "172.20.10.4";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
        printf("Connection error.");
        return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
        printf("Fail to connect MQTT\r\n");
    }
      if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
        printf("Fail to subscribe\r\n");
    }
	mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
  	/***************************/

   while (true) {
    button.rise(&start_tilt);
    if (confirm){
        printf("angle confirm, angle = %d\n", angle_threshold);
        UIled = 0;
		angle_key = false;
        //thread_UI.join(); // not close !!!!
        //thread_UI.terminate();
        //printf("thread_UI.join()\n");
		mqtt_queue.event(&publish_message, &client);
        confirm = false;
    }
    memset(buf, 0, 256);      // clear buffer
    for(int i=0; i<255; i++) {
        char recv = fgetc(devin);
        if (recv == '\r' || recv == '\n') {
            printf("\r\n");
            break;
        }
        buf[i] = fputc(recv, devout);
    }
    RPC::call(buf, outbuf);
    printf("%s\r\n", outbuf);
   }
	/****************************/
   	int num = 0;
    while (num != 5) {
        client.yield(100);
        ++num;
    }

    while (1) {
        if (closed) break;
        client.yield(500);
        ThisThread::sleep_for(500ms);
    }

    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
        printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    	printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
	/*********************************/

    return 0;
}

void start_tilt(){
    confirm = true;
}

void tilt_angle(Arguments *in, Reply *out) {
	angle_key = true;
    thread_tilt.start(&detect_angle);
    printf("tilt_angle RPC is triggered\n");
}

void detect_angle(){
    int16_t ref_pDataXYZ[3] = {0, 0, 0};    //ref value
    int16_t pDataXYZ[3],arcDataXYZ[3] = {0};
    double theta = 0, cos_theta = 0;
    //char buffer[200];

    //set ref acc value 
    initled = 1;
    ThisThread::sleep_for(2s);
    BSP_ACCELERO_AccGetXYZ(ref_pDataXYZ);
    initled = 0;

    //start detect angle 
    tiltled = 1;
    // printf("value of ref_pDataXYZ = %d, %d, %d\n", ref_pDataXYZ[0], ref_pDataXYZ[1], ref_pDataXYZ[2]);
    
    while (angle_key){
        BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        // printf("%value of pDataXYZ = %d, %d, %d\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
        for (int i = 0; i < 3; i++)
            arcDataXYZ[i] = pDataXYZ[i]- ref_pDataXYZ[i];
        // printf("value of arcDataXYZ = %d, %d, %d\n", arcDataXYZ[0], arcDataXYZ[1], arcDataXYZ[2]);

        // printf("absolute value of ref_pDataXYZ = %lf\n", length(ref_pDataXYZ));
        // printf("absolute value of pDataXYZ = %lf\n", length(pDataXYZ));
        // printf("absolute value of arcpDataXYZ = %lf\n", length(arcDataXYZ));
        // printf("square of absolute value of ref_pDataXYZ = %lf\n", pow(length(ref_pDataXYZ), 2));
        // printf("square of absolute value of pDataXYZ = %lf\n", pow(length(pDataXYZ), 2));
        // printf("square of absolute value of arcpDataXYZ = %lf\n", pow(length(arcDataXYZ), 2));

        cos_theta = (square(length(ref_pDataXYZ)) + square(length(pDataXYZ)) - square(length(arcDataXYZ))) / (2 * length(ref_pDataXYZ) * length(pDataXYZ));
        //printf("cos_theta: %lf\n", cos_theta);

        if (cos_theta <= 1.00 && cos_theta >= -1.00){
            theta = acos(cos_theta) * 180 / 3.141593;
            printf("angle: %lf\n", theta);
        }
        else if (cos_theta > 1.00){
            theta = acos(1.00) * 180 / 3.141593;
            printf("angle: %lf\n", theta);
        }
        else if (cos_theta < -1.00){
            theta = acos(-1.00) * 180 / 3.141593;
            printf("angle: %lf\n", theta);
        }
        if (theta > angle_threshold){
            tiltled = 0;
			angle_key = false;
        }
    }
    
}

inline double length(int16_t * DataXYZ){
    double x, y, z, ans;
    x = double(DataXYZ[0])/100;
    y = double(DataXYZ[1])/100;
    z = double(DataXYZ[2])/100;
    ans = square(x) + square(y) + square(z);
    return ans;
}
inline double square(double x){
    return x * x;
};
/////**********************************/////
void gesture_UI(Arguments *in, Reply *out) {
    UIled = 1;
	gesture_key = true;
    thread_UI.start(&detect_gesture);
    printf("gesture_UI RPC is triggered\n");
}

int detect_gesture(){
  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return -1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return -1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return -1;
  }

  error_reporter->Report("Set up successful...\n");

  while (gesture_key) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
        //error_reporter->Report(config.output_message[gesture_index]);
        if (angle_threshold == 180)
            angle_threshold = 30;
        else 
            angle_threshold += 30;
        printf("angle_threshold = %d\n", angle_threshold);
    }
  }
}


// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}
/////**********************************/////

/**********************************/
void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    message_num++;
    MQTT::Message message;
    char buff[100];
    sprintf(buff, "QoS0 Hello, Python! #%d", message_num);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void close_mqtt() {
    closed = true;
}
/**********************************/
