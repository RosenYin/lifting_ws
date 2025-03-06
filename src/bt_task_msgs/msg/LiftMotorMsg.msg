uint8 motorId
bool initState

uint8 ctrlMode
uint8 backVol
float64 backCurrent

uint8 stateBit
bool runState
bool overflowI
bool overflowV
bool encodeErr
bool posBiasOver
bool underVol
bool overLoad
bool externalCtrlMode

int16 motorSpeed
int16 backSpeed
uint8 locationComplete
int32 backPos
int32 backHeight
int64 targetHeight
int64 targetPos
bool upLimit
bool downLimit

bool reachTargetPos
bool stopFlag
int64 fpsError