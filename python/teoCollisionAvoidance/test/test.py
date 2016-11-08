import openravepy
from openravepy import *
import numpy


def main(env, options):
    """Main example code"""
    samplingdelta = options.samplingdelta
    RaveLoadPlugin('./YarpRobot')
    # Load the teo robot
    env.Load(options.target)
    teo_robot = env.GetRobots()[0]


    # Open YarpRobot plugin ports
    YarpRobot = RaveCreateModule(env,'YarpRobot')
    print YarpRobot.SendCommand('open')

    print 'The collision mesh normal robot (no CD) is'
    print teo_robot.GetLinks()[1].GetGeometries()[0].GetCollisionMesh()

    # We move the robot to a selfcollision position as a test
    teo_robot.SetDOFValues([0, 0, 0, 0, 0,
                            0.1, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                            0, 0, 0, 0, 0,
                            0, 0, 0])

    # print teo_robot.GetDOFValues()

    # ConvexDecomposition class
    cdmodel = databases.convexdecomposition.ConvexDecompositionModel(teo_robot)
    print(cdmodel.getfilename())

    # Load the ConvexDecomposition model, if it does not exit in the database generate it.
    # if not cdmodel.load():
        # cdmodel.autogenerate()
        # If not already in the database. Generate
    #cdmodel.generate(padding=0.02)

    print 'The collision mesh normal robot is'
    print teo_robot.GetLinks()[1].GetGeometries()[0].GetCollisionMesh()

    #cdmodel.save()
    print 'Finished saving'

    # WARNING show stops the program
    # cdmodel.show()

    # setrobot sets the ConvexDecomposition model as the collision mesh of the robot.
    print 'Setting robot...'
    #cdmodel.setrobot()
    print 'Finish setrobot'

    print 'The collision mesh of the CD robot'
    print teo_robot.GetLinks()[1].GetGeometries()[0].GetCollisionMesh()

    # print cdmodel.robot.GetNonAdjacentLinks()

    print 'Colliding?'
    if teo_robot.CheckSelfCollision():
        print 'Im self colliding yeah modafaka!!'
    else:
        print 'No self collision detected'

    while 1:
        pass


from optparse import OptionParser
from openravepy.misc import OpenRAVEGlobalArguments


def run(args=None):
    """Command-line execution of the example.

    :param args: arguments for script to parse, if not specified will use sys.argv
    """
    parser = OptionParser(
        description='Builds the convex decomposition of the robot and plots all the points that are tested inside of it.')
    OpenRAVEGlobalArguments.addOptions(parser)
    parser.add_option('--target', action="store", type='string', dest='target',
                      default='/home/raul/repos/teo-main/share/openrave/teo/teo.robot.xml',
                      help='Target body to load (default=%default)')
    parser.add_option('--samplingdelta', action="store", type='float', dest='samplingdelta', default=None,
                      help='The sampling rate for the robot (default=%default)')
    (options, leftargs) = parser.parse_args(args=args)
    OpenRAVEGlobalArguments.parseAndCreateThreadedUser(options, main, defaultviewer=True)


if __name__ == "__main__":
    run()
