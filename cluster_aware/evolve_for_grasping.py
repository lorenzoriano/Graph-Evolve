import sys
sys.path.append('\\\\isrchn1\\userdata\\se15005594\\Graph-Evolve\\src')

from pyevolve import GSimpleGA
from pyevolve import G1DList
from pyevolve import Selectors
from pyevolve import Initializators, Mutators,  Consts
from pyevolve import DBAdapters
from pyevolve import Crossovers
from pyevolve import Consts

import graph_evolve
from graph_evolve import graph_genome
import smach
from graph_evolve import smach_grasp
from graph_evolve.chromosome_smach import convert_chromosome_smach

import cPickle
from mpi4py import MPI
from pyevolve import mpi_migration

smach.set_loggers(smach.loginfo,
                  smach_grasp.null_print,
                  smach_grasp.null_print,
                  smach_grasp.null_print)

names_mapping = ["RecogniseObject",
                 "MoveToObject",
                 "GraspObject",
                 "CheckSuccess",
                 "MoveGripperToParam"]
classes_mapping = {"RecogniseObject" : smach_grasp.RecogniseObject,
                 "MoveToObject": smach_grasp.MoveGripperTo,
                 "GraspObject": smach_grasp.GraspObject,
                 "CheckSuccess": smach_grasp.CheckSuccess,
                 "MoveGripperToParam" : smach_grasp.MoveGripperToParam}
transitions_mapping = {'RecogniseObject':["success",],
                       'MoveToObject':["success", "failure",],
                       'GraspObject':["success", "failure",],
                       "CheckSuccess":[],
                       "MoveGripperToParam":["success", "failure"]}

default_mapping = {'state_in':'robot_state',
                   'state_out':'robot_state',
                   }
data_mapping = {'RecogniseObject': default_mapping,
                'MoveToObject': default_mapping,
                'GraspObject' :default_mapping,
                "CheckSuccess": default_mapping,
                "MoveGripperToParam": default_mapping}
 
node_degrees = [1,2,2,0,2]
nodes_params = [0,0,0,0,3]

def single_try(chromosome, **args):
    sm = convert_chromosome_smach(chromosome, 
                                  names_mapping, 
                                  transitions_mapping, 
                                  classes_mapping,
                                  data_mapping)
    robot_state =  smach_grasp.RobotWorldState()
    sm.userdata.robot_state = robot_state
    outcome = sm.execute()
    
    if outcome == "success":
#        print "Great!"
        return 0
    else:
#        dist = smach_grasp.state_dist_gripper_object(sm.userdata.robot_state)
#        return dist
        if sm.userdata.robot_state.object_in_gripper:
            return smach_grasp.state_dist_gripper_object(sm.userdata.robot_state,
                                                         (0.0,0.0,0.0))
        else:
            return 25

def eval_func(chromosome, **args):
    res = []
    for _ in xrange(100): 
        res.append(single_try(chromosome, **args))
    return sum(res)


freq_stats = 10

def stepCallback(ga_engine):
    generation = ga_engine.getCurrentGeneration()
    if generation % freq_stats == 0:

        comm = MPI.COMM_WORLD
        if comm.rank == 0:
            migrator = ga_engine.migrationAdapter
            if migrator.all_stars is not None:
                
                print "All stars: ", [i.score for i in migrator.all_stars]
                best = max(migrator.all_stars)
                file = open("bestgenome.txt","w")
                cPickle.dump(best, file)
                file.close()
                smach.set_loggers(smach.loginfo,
                        smach.logwarn,
                        smach_grasp.null_print,
                        smach.logerr)
                score = eval_func(best)
                print "Fitness: ", best.score
                print "Real Score: ", score
                print
                ga_engine.printTimeElapsed()
        
        
    return False

if __name__ == "__main__":

    comm = MPI.COMM_WORLD
    
    smach.set_loggers(smach_grasp.null_print,
                      smach.logwarn,
                      smach_grasp.null_print,
                      smach.logerr)
    num_nodes= 5
    pop_size = 500
    poolsize = int(pop_size / 10.)
    migration_size = 5
    migration_rate = 50
    elitism_size = 5
    generations = 1000

    if comm.rank == 0:
        print "Num nodes: ", num_nodes
        print "Pop Size: ", pop_size
        print "Tournament Size: ", poolsize
        print "Migration Size: ", migration_size
        print "Elitism Size: ", elitism_size
        print "Number of Generations: ", generations
    
    genome = graph_genome.GraphGenome(num_nodes, node_degrees, nodes_params)
    genome.evaluator.set(eval_func)
    
    ga = GSimpleGA.GSimpleGA(genome)
#    print "STOPPING ELITISM"
    ga.setElitism(True)
    ga.setElitismReplacement(1)
    
    ga.selector.set(Selectors.GRouletteWheel)
    ga.setSortType(Consts.sortType["raw"])
    
    ga.setGenerations(generations)
    ga.setPopulationSize(pop_size)
    ga.setCrossoverRate(0.2)
    ga.setMutationRate(0.2)
    ga.getPopulation().setParams(tournamentPool = poolsize)
    genome.setParams(p_del=0.2, p_add=0.2)
#    ga.setMinimax(Consts.minimaxType["maximize"])
    ga.setMinimax(Consts.minimaxType["minimize"])

    if comm.size > 1:
        migrator = mpi_migration.MPIMigrator()
        migrator.setGAEngine(ga)
        migrator.setNumReplacement(migration_size)
        migrator.setMigrationRate(migration_rate)
        migrator.selector.set(Selectors.GRankSelector)

        ga.setMigrationAdapter(migrator)
    

    # Do the evolution
    if comm.rank == 0:
        ga.stepCallback.set(stepCallback)
        ga.evolve(freq_stats = freq_stats)
    else:
        ga.evolve(freq_stats = 0)

    if comm.rank == 0:
        # Best individual
        best = ga.bestIndividual()

        file = open("bestgenome.txt","w")
        cPickle.dump(best, file)
        file.close()

        sm = convert_chromosome_smach(best,
                                    names_mapping,
                                    transitions_mapping,
                                    classes_mapping,
                                    data_mapping)
        robot_state =  smach_grasp.RobotWorldState()
        sm.userdata.robot_state = robot_state

        smach.set_loggers(smach.loginfo,
                        smach.logwarn,
                        smach_grasp.null_print,
                        smach.logerr)
        try:
            outcome = sm.execute()
        except smach.exceptions.InvalidUserCodeError as err:
            print "Error: ", err.message
        else:
            print "Outcome: ", outcome
        print "Distance is: ", smach_grasp.state_dist_gripper_object(sm.userdata.robot_state,
                                                                    (0.0,5.0,0.0))

        print "eval is: ", eval_func(best)
    
    
    