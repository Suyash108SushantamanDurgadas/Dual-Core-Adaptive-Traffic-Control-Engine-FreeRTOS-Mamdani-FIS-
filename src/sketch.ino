#include <ESP32Servo.h>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <string>
#include <algorithm>

using std::vector;
using std::string;

// ---------------- HARDWARE & SIGNAL MAPS ---------------- //
int traffic_light_map[4][3] = {
  {2,15,4},
  {16,17,5},
  {18,19,21},
  {12,14,27}
};
int cam_servo_map[2] = {22,23};
const char dir[4] = {'E','N','W','S'};
char signal_map[3] = {'R','Y','G'};
int signaltime[3] = {100000, 5000, 20000}; 
int cam_lane_montime = 2500;

// ---------------- FUZZY DATA TYPES ---------------- //
struct Membershipfn {
    float higherlimit;
    float lowerlimit;
    float peakval;
    float leastval;
    float peakmfval;
};

struct Membershipval {
    float valarr[2];
    string membdescp[2];
    int mfindex[2];
};

struct Opmfval {
    float opmf;
    string opmfdescp;
    int max_mfindex;
}; 

typedef struct {
  int numvehicles;
  int signaltimes[3];
  char laneorinetation;
  char currentsignal;
  int laneno;
  int nextlaneveh;
} LaneData;

typedef struct {
  int numvehicles[4];
  int stepsize = 90;
  int camservoangles[3] = {0, 90, 180};
  int baseservoangles[2] = {0, 90};
  bool camservotrans[4] = {0, 1, 0, 1};
  bool baseservotrans[4] = {0, 0, 1, 0};
  int camoperation_time;
} JunCamData;

// ---------------- FUZZY LOGIC ENGINE CLASSES ---------------- //
class Fuzzy {
private:
    int numdescp;
    vector<string> descp;
    vector<Membershipfn> membershipfn;
    int maxcrispip;
    int mincrispip;
    vector<float> supportpts;
    float step;
    Membershipval membwrap;

public:
    Fuzzy(int numdescp, const vector<string> &descp, int maxcrispip, int mincrispip)
        : numdescp(numdescp), descp(descp), maxcrispip(maxcrispip), mincrispip(mincrispip), supportpts(numdescp) 
    {
        membershipfn.resize(numdescp); 
        createTriangularmf();
    }

    void createTriangularmf() {
        supportpts[0] = static_cast<float>(mincrispip);
        supportpts[numdescp - 1] = static_cast<float>(maxcrispip);
        step = static_cast<float>(maxcrispip - mincrispip) / (numdescp - 1);

        for (int i = 1; i < numdescp - 1; i++) {
            supportpts[i] = supportpts[0] + step * i;
        }
        for (int i = 0; i < numdescp; i++) {
            membershipfn[i].peakval = 1;
            membershipfn[i].peakmfval = supportpts[i]; 
        }
        for (int i = 1; i < numdescp; i++) {
            membershipfn[i].lowerlimit = supportpts[i - 1];
        }
        membershipfn[0].lowerlimit = supportpts[0];
        for (int i = 0; i < numdescp - 1; i++) {
            membershipfn[i].higherlimit = supportpts[i + 1];
        }
        membershipfn[numdescp - 1].higherlimit = supportpts[numdescp - 1];
    }

    void findMFval(int valnum) {
        int index = valnum / static_cast<int>(step);
        if (index >= numdescp - 1) index = numdescp - 2;

        membwrap.valarr[0] = ((valnum - membershipfn[index + 1].lowerlimit) / (membershipfn[index + 1].peakmfval - membershipfn[index + 1].lowerlimit));
        membwrap.valarr[1] = ((membershipfn[index].higherlimit - valnum) / (membershipfn[index].higherlimit - membershipfn[index].peakmfval));
        membwrap.membdescp[0] = descp[index + 1];
        membwrap.membdescp[1] = descp[index];
        membwrap.mfindex[0] = index + 1;
        membwrap.mfindex[1] = index;
    }

    Membershipval getmembwrap() { return membwrap; }
};

class Defuzzification {
private:
    int numopdescp;
    int peaktimeval;
    vector<float> supportpts;
    string opmfdescp[3] = {"Low", "Medium", "High"};
    vector<Membershipfn> opmembfn;
    float opcrispval;
    float step;
      
public:
    Defuzzification(int numopdescp, int peaktimeval) {
        this->numopdescp = numopdescp;
        this->peaktimeval = peaktimeval;
        supportpts.resize(numopdescp);
    }
      
    void mfcreation() {
        step = static_cast<float>(peaktimeval) / static_cast<float>(numopdescp - 1);
        supportpts[0] = 0;
        supportpts[1] = step;
        supportpts[2] = static_cast<float>(peaktimeval);
        opmembfn.resize(numopdescp);
        
        for (int i = 0; i < numopdescp; i++) {
            opmembfn[i].peakmfval = supportpts[i];
            opmembfn[i].peakval = 1;
            opmembfn[i].leastval = 0;
        }
        for (int i = 0; i < numopdescp - 1; i++) {
            opmembfn[i].higherlimit = supportpts[i + 1];
        }
        opmembfn[2].higherlimit = supportpts[2];
        for (int i = 1; i < numopdescp; i++) {
            opmembfn[i].peakmfval = supportpts[i - 1];
        }
        opmembfn[0].peakmfval = supportpts[0];
    }
      
    float defuzzifiedop(float opmfval, string opdescp) {
        int matched_index = 0;
        for (int i = 0; i < numopdescp; i++) {
            if (opmfdescp[i] == opdescp) {
                matched_index = i;
            }
        }
        if (matched_index == 2) {
            opcrispval = opmembfn[matched_index].lowerlimit + opmfval * (opmembfn[matched_index].higherlimit - opmembfn[matched_index].lowerlimit);
        } else if (matched_index == 0) {
            opcrispval = opmembfn[matched_index].lowerlimit + (1 - opmfval) * (opmembfn[matched_index].higherlimit - opmembfn[matched_index].lowerlimit);
        } else {
            opcrispval = step;
        }
        return opcrispval;
    }
};

class CarFIS {
private:
    int numopdescp;
    string traffic_time_map[4][4];
    int xindex[2];
    int yindex[2];
    string opdescp[4];
    float opvals[4];
    Opmfval op;
    Defuzzification DefuzedOp;
    float crispval;
      
public:
    CarFIS(int numopdescp, int peaktimeval) : DefuzedOp(numopdescp, peaktimeval) { 
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                if (i == j) {
                    traffic_time_map[i][j] = "Medium"; 
                } else if (i < j) {
                    traffic_time_map[i][j] = "High";
                } else {
                    traffic_time_map[i][j] = "Low";
                }
            }
        }
        this->numopdescp = numopdescp;
    }

    void getopmfval(Membershipval &membershipval1, Membershipval &membershipval2) {
        xindex[0] = membershipval1.mfindex[0];
        xindex[1] = membershipval1.mfindex[1];
        yindex[0] = membershipval2.mfindex[0];
        yindex[1] = membershipval2.mfindex[1];
        
        opdescp[0] = traffic_time_map[xindex[0]][yindex[0]];
        opdescp[1] = traffic_time_map[xindex[0]][yindex[1]];
        opdescp[2] = traffic_time_map[xindex[1]][yindex[0]];
        opdescp[3] = traffic_time_map[xindex[1]][yindex[1]];
         
        opvals[0] = std::min(membershipval1.valarr[0], membershipval2.valarr[0]);
        opvals[1] = std::min(membershipval1.valarr[0], membershipval2.valarr[1]);
        opvals[2] = std::min(membershipval1.valarr[1], membershipval2.valarr[0]);
        opvals[3] = std::min(membershipval1.valarr[1], membershipval2.valarr[1]);
          
        op.opmf = opvals[0];
        op.max_mfindex = 0;
        for (int i = 1; i < 4; i++) {
            if (op.opmf < opvals[i]) {
                op.opmf = opvals[i];
                op.max_mfindex = i;
            }
        }
        op.opmfdescp = opdescp[op.max_mfindex];
    }
      
    Opmfval getOpfuzzyval() { return op; }
    
    float getCrispval() {
        DefuzedOp.mfcreation();
        crispval = DefuzedOp.defuzzifiedop(op.opmf, op.opmfdescp);
        return crispval;
    }
};

// ---------------- SYSTEM HARDWARE CLASSES ---------------- //
class Lane {
  private:
    LaneData lanedata;
    int signal_pins[3];
  public:
    Lane(int laneno) {
      lanedata.laneno = laneno;
      for (int j = 0; j < 3; j++) {
        signal_pins[j] = traffic_light_map[laneno][j];
        lanedata.signaltimes[j] = signaltime[j]; 
        pinMode(signal_pins[j], OUTPUT);
      }
      digitalWrite(signal_pins[0], HIGH);
      digitalWrite(signal_pins[1], LOW);
      digitalWrite(signal_pins[2], LOW);
    }
    
    void traffic_functioning() {
      Serial.print("\n=== Active Phase: Lane "); Serial.print(lanedata.laneno);
      Serial.print(" ("); Serial.print(dir[lanedata.laneno]); Serial.println(") ===");
      
      // Dynamic Green
      Serial.print("  -> GREEN for: "); Serial.print(lanedata.signaltimes[2] / 1000.0f); Serial.println("s");
      set_greensignal();
      vTaskDelay(pdMS_TO_TICKS(lanedata.signaltimes[2])); 
      
      // Yellow Clearance
      Serial.print("  -> YELLOW for: "); Serial.print(lanedata.signaltimes[1] / 1000.0f); Serial.println("s");
      set_yellowsignal();
      vTaskDelay(pdMS_TO_TICKS(lanedata.signaltimes[1]));
      
      set_redsignal();
    }
    
    void set_redsignal() {
      lanedata.currentsignal = signal_map[0];
      digitalWrite(signal_pins[0], HIGH);
      digitalWrite(signal_pins[1], LOW);
      digitalWrite(signal_pins[2], LOW);
    }
    void set_yellowsignal() {
      lanedata.currentsignal = signal_map[1];
      digitalWrite(signal_pins[1], HIGH);
      digitalWrite(signal_pins[0], LOW);
      digitalWrite(signal_pins[2], LOW);
    }
    void set_greensignal() {
      lanedata.currentsignal = signal_map[2];
      digitalWrite(signal_pins[2], HIGH);
      digitalWrite(signal_pins[1], LOW);
      digitalWrite(signal_pins[0], LOW);
    }
    
    char get_currentsignal() { return lanedata.currentsignal; }
    LaneData getlanedata() { return lanedata; }
    void setnumvehicles(int num_vehicle) { lanedata.numvehicles = num_vehicle; }
    
    int get_next_lane_veh_num() {
      lanedata.nextlaneveh = rand() % 100;
      return lanedata.nextlaneveh;
    }
    
    void update_greentime(int calculated_ms) {
      lanedata.signaltimes[2] = calculated_ms;
    }
};

class Cam_Op {
  private:
    JunCamData Camdata;
    int signal_pins[2];
    Servo CamServo;
    Servo BaseServo;
    int cam_currentang = 0;
    int base_currentang = 0; 

  public:
    Cam_Op() {
      for (int j = 0; j < 2; j++) {
        signal_pins[j] = cam_servo_map[j];
        pinMode(signal_pins[j], OUTPUT);
      }
      CamServo.attach(signal_pins[0]);
      BaseServo.attach(signal_pins[1]);
      Camdata.camoperation_time = cam_lane_montime;
    }
    
    void cam_monitoring() {
      Serial.println("\n--- Starting Camera Sweep Across Junctions ---");
      for (int i = 0; i < 4; i++) {
        cam_currentang = cam_currentang + Camdata.camservotrans[i] * Camdata.stepsize;
        base_currentang = base_currentang + Camdata.baseservotrans[i] * Camdata.stepsize;
        CamServo.write(cam_currentang);
        BaseServo.write(base_currentang); 
        
        get_numvehicles(i);
        
        Serial.print("  Camera pointing at Lane "); Serial.print(i);
        Serial.print(" ("); Serial.print(dir[i]); Serial.print(") angles -> Base: ");
        Serial.print(base_currentang); Serial.print(", Cam: "); Serial.print(cam_currentang);
        Serial.print(" | Detected Vehicles: "); Serial.println(Camdata.numvehicles[i]);
        
        vTaskDelay(pdMS_TO_TICKS(Camdata.camoperation_time));
      }
      cam_currentang = 0;
      base_currentang = 0;
      Serial.println("--- Camera Sweep Completed & Reset to East ---");
    }
    
    void get_numvehicles(int i) {
      Camdata.numvehicles[i] = rand() % 100;
    }
    JunCamData Camdatawrap() { return Camdata; }
    int get_lane_veh_num(int i) { return Camdata.numvehicles[i]; }
};

typedef struct {
  LaneData lanedata[4];
  JunCamData camdata; 
  int JunctionId;
} JunctionData;

class Junction {
  private:
    Lane JunLanes[4];
    Cam_Op JunCam;
    JunctionData CurrentJundata; 
       
  public:
    Junction(int Junid) : JunLanes{Lane(0), Lane(1), Lane(2), Lane(3)}, JunCam() {
      CurrentJundata.JunctionId = Junid;
    }
    
    void Juntraffic_op() {
      for (int i = 0; i < 4; i++) {
        JunLanes[i].traffic_functioning();
      }

      run_adaptive_routing();
      setJunctiondata();
    }
    
    void Juncam_op() {
      JunCam.cam_monitoring();
    }
    
    void run_adaptive_routing() {
      vector<string> labels = {"Low", "Medium", "High", "V.High"};
      Serial.println("\n==================================================");
      Serial.println("  RE-CALCULATING TRAFFIC TIMINGS VIA FUZZY CORES  ");
      Serial.println("==================================================");
      
      for (int i = 0; i < 4; i++) {
        int current_veh = JunCam.get_lane_veh_num(i);
        int next_veh = JunLanes[i].get_next_lane_veh_num();
        
        JunLanes[i].setnumvehicles(current_veh);

        Fuzzy fCurrent(4, labels, 100, 0);
        Fuzzy fNext(4, labels, 100, 0);
        
        fCurrent.findMFval(current_veh);
        fNext.findMFval(next_veh);
        
        Membershipval mfC = fCurrent.getmembwrap();
        Membershipval mfN = fNext.getmembwrap();
        
        CarFIS fisEngine(3, 45); 
        fisEngine.getopmfval(mfC, mfN);
        
        float adaptive_seconds = fisEngine.getCrispval();
        int adaptive_ms = static_cast<int>(adaptive_seconds * 1000.0f);
        
        // Guard minimum bound logic
        if (adaptive_ms < 5000) {
          adaptive_ms = 5000;
          adaptive_seconds = 5.0f;
        }
        
        JunLanes[i].update_greentime(adaptive_ms);
        Serial.print("  [Lane "); Serial.print(i); Serial.print(" ("); Serial.print(dir[i]); Serial.println(")]");
        Serial.print("    -> Current Vehicles (Sensor): "); Serial.println(current_veh);
        Serial.print("    -> Upstream Projected Inflow : "); Serial.println(next_veh);
        Serial.print("    -> Committed Next Green Time : "); Serial.print(adaptive_seconds); Serial.println("s");
        Serial.println("  ---");
      }
      Serial.println("==================================================\n");
    }
    
    void setJunctiondata() {
      CurrentJundata.camdata = JunCam.Camdatawrap(); 
      for (int i = 0; i < 4; i++) {
        CurrentJundata.lanedata[i] = JunLanes[i].getlanedata(); 
      }
    }
};

// Global Instance
Junction SubwayJunction(1);

void Traffic_Signal(void *pvParameters);
void Traffic_Monitoring(void *pvParameters);

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial monitor time to settle
  Serial.println("Initializing Adaptive Subterranean Traffic Core...");
  srand(analogRead(0)); 

  xTaskCreatePinnedToCore(Traffic_Signal, "Signal", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Traffic_Monitoring, "Monitoring", 4096, NULL, 1, NULL, 0);
}

void Traffic_Signal(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    SubwayJunction.Juntraffic_op();
  }
}

void Traffic_Monitoring(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    SubwayJunction.Juncam_op();
    SubwayJunction.setJunctiondata();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void loop() {
}
