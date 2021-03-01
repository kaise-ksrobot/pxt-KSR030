#include "pxt.h"
using namespace pxt;
namespace KSRobotCPP
{

    //%
    int mb_version()
    {
#if MICROBIT_CODAL
        return 0;
#endif
        return 1;
    }

} // namespace KSRobotCPP
