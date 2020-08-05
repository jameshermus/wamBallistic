/*** A Libbarrett Real-Time System Example Program ***/
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/standard_main_function.h>

#include <barrett/math/matrix.h>
#include <iostream>

typedef typename ::barrett::math::Matrix<4,4> JacobM;

using namespace barrett;
template<size_t DOF>
class ExampleSystem : public systems::System
{
 BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
 public:
 Input<jt_type> commandedJTIn;
 Input<jp_type> wamJPIn;
 Output<jt_type> wamJTOutput;
 JacobM wamStiff;
 JacobM wamDamp;
 protected:
 typename Output<jt_type>::Value* jtOutputValue;
 public:
 ExampleSystem(const std::string& sysName = "ExampleSystem") :
 System(sysName), commandedJTIn(this), wamJPIn(this),
 wamJTOutput(this, &jtOutputValue)
 {
   wamStiff(0,0) = 50;
   wamStiff(2,2) = 50;
   wamDamp = wamStiff * 0.1;
   std::cout<<wamDamp<<std::endl;
     // just a line to show if overwirte works
}
 virtual ~ExampleSystem()
 {
 this->mandatoryCleanUp();
 }
 protected:
 virtual void operate()
 {
 commandedJT = commandedJTIn.getValue();
 wamJP = wamJPIn.getValue();
 wamJP[0] = 1;
 printf("wam JP is %2.2f", wamJP[0]);
 // If we were to do any real-time control calculations they
 // would take place here.
 jtOutputValue->setData(&commandedJT);
 }
 public:
 jt_type commandedJT;
 jp_type wamJP;
 private:
 DISALLOW_COPY_AND_ASSIGN(ExampleSystem);
};
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
 systems::Wam<DOF>& wam)
{
 BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
 // Turn on Gravity Compensation
 wam.gravityCompensate();
 // Create an instance of our system
 ExampleSystem<DOF> exampleSystem;
 // Connect inputs
 systems::connect(wam.jtSum.output, exampleSystem.commandedJTIn);
 systems::connect(wam.jpOutput, exampleSystem.wamJPIn);
 
 //systems::connect(exampleSystem.commandedJT, wam.jtinput);
 printf("Press <Enter> to move to the zero position for each joint.\n");
 detail::waitForEnter();
 jp_type zeroMove; // Defaults to all zero values
 // Normally moveTo calls are blocking, and the rest of your
 // program will not continue until the move is done. Passing
 // false will make this a non-blocking movement, allowing us to
 // monitor and print.
 wam.moveTo(zeroMove, false);
 while (!wam.moveIsDone())
 {
 if (DOF == 4)
 {
      printf("Current WAM Joint Positions- J1: %f, J2: %f, J3: %f, J4: %f\n",
 exampleSystem.wamJP[0], exampleSystem.wamJP[1],
 exampleSystem.wamJP[2], exampleSystem.wamJP[3]);
// printf("Commanded WAM Joint Torques- J1: %f, J2: %f, J3: %f, J4: %f\n",
// exampleSystem.commandedJT[0], exampleSystem.commandedJT[1],
// exampleSystem.commandedJT[2], exampleSystem.commandedJT[3]);
 }
 else
 {
 printf("Current WAM Joint Positions- J1: %f, J2: %f, J3: %f, J4: %f, J5:%f, J6: %f, J7: %f\n",
 exampleSystem.wamJP[0], exampleSystem.wamJP[1],
 exampleSystem.wamJP[2], exampleSystem.wamJP[3],
 exampleSystem.wamJP[4], exampleSystem.wamJP[5],
 exampleSystem.wamJP[6]);
 printf("Commanded WAM Joint Torques- J1: %f, J2: %f, J3: %f, J4: %f, J5:%f, J6: %f, J7: %f\n",
 exampleSystem.commandedJT[0], exampleSystem.commandedJT[1],
 exampleSystem.commandedJT[2], exampleSystem.commandedJT[3],
 exampleSystem.commandedJT[4], exampleSystem.commandedJT[5],
 exampleSystem.commandedJT[6]);
 }
 }
 printf("Finished Zero Move");
 printf("Press <Enter> to move WAM back to home position.\n");
 detail::waitForEnter();
 wam.moveHome(false);
 while (!wam.moveIsDone())
 {
 if (DOF == 4)
 {
 printf("Current WAM Joint Positions- J1: %f, J2: %f, J3: %f, J4: %f\n",
 exampleSystem.wamJP[0], exampleSystem.wamJP[1],
 exampleSystem.wamJP[2], exampleSystem.wamJP[3]);
 //printf("Commanded WAM Joint Torques- J1: %f, J2: %f, J3: %f, J4: %f\n",
 //exampleSystem.commandedJT[0], exampleSystem.commandedJT[1],
 //exampleSystem.commandedJT[2], exampleSystem.commandedJT[3]);
 }
 else
 {
 printf("Current WAM Joint Positions- J1: %f, J2: %f, J3: %f, J4: %f, J5:%f, J6: %f, J7: %f\n",
 exampleSystem.wamJP[0], exampleSystem.wamJP[1],
 exampleSystem.wamJP[2], exampleSystem.wamJP[3],
 exampleSystem.wamJP[4], exampleSystem.wamJP[5],
 exampleSystem.wamJP[6]);
 printf("Commanded WAM Joint Torques- J1: %f, J2: %f, J3: %f, J4: %f, J5:%f, J6: %f, J7: %f\n",
 exampleSystem.commandedJT[0], exampleSystem.commandedJT[1],
 exampleSystem.commandedJT[2], exampleSystem.commandedJT[3],
 exampleSystem.commandedJT[4], exampleSystem.commandedJT[5],
 exampleSystem.commandedJT[6]);
 }
 }
  printf("Finished Home Move");
 pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
 return 0;
}
