import openravepy
import numpy as np

class HerbRobot(object):

    def __init__(self, env, manip):
        self.name = 'herb'
        self.robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
        env.Add(self.robot)

        # relaxed positions
        right_relaxed = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
        left_relaxed = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]

        # Require numpy 1.8.x
        right_manip = self.robot.GetManipulator('right_wam')
        self.robot.SetActiveDOFs(right_manip.GetArmIndices())
        self.robot.SetActiveDOFValues(right_relaxed)

        left_manip = self.robot.GetManipulator('left_wam')
        self.robot.SetActiveDOFs(left_manip.GetArmIndices())
        self.robot.SetActiveDOFValues(left_relaxed)

        if manip == 'right':
            self.manip = right_manip
            self.robot.SetActiveManipulator('right_wam')
        else:
            self.manip = left_manip
            self.robot.SetActiveManipulator('left_wam')

        self.robot.SetActiveDOFs(self.manip.GetArmIndices())

        self.robot.controller = openravepy.RaveCreateController(self.robot.GetEnv(), 'IdealController')

    def GetCurrentConfiguration(self):
        return self.robot.GetActiveDOFValues()

    def ConvertPlanToTrajectory(self, plan):

     suc = ""
     oplan = np.array(plan)
     n = 1
     while not suc.lower().strip() == "hassolution":
         # Create a trajectory
         traj = openravepy.RaveCreateTrajectory(self.robot.GetEnv(), 'GenericTrajectory')
         config_spec = self.robot.GetActiveConfigurationSpecification()
         traj.Init(config_spec)
         new_plan = np.zeros(((oplan.shape[0] - 1)*n, oplan.shape[1]), dtype=oplan.dtype)
         for i, r in enumerate(np.linspace(0, 1.0, n + 1)[:-1]):
             new_plan[i::n, ...] = oplan[:-1, ...]*(1 - r) + oplan[1:, ...]*r
             plan = new_plan

         idx = 0
         for pt in plan:
             traj.Insert(idx, pt)
             idx = idx + 1

         suc = str(openravepy.planningutils.RetimeActiveDOFTrajectory(traj, self.robot, maxvelmult=1,maxaccelmult=1, hastimestamps=False, plannername='ParabolicTrajectoryRetimer'))
         n *= 2

     return traj


    def ExecuteTrajectory(self, traj):

        # Send the trajectory to the controller and wait for execution to complete
        self.robot.GetController().SetPath(traj)
        self.robot.WaitForController(0)
