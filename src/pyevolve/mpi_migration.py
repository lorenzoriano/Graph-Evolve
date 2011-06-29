from mpi4py import MPI
from FunctionSlot import FunctionSlot
import Consts
from random import choice as rand_choice

class MPIMigrator(object):
    selector = None
    """ This is the function slot for the selection method
    if you want to change the default selector, you must do this: ::

    migration_scheme.selector.set(Selectors.GRouletteWheel) """

    def __init__(self):
        self.myself = None
        self.selector = FunctionSlot("Selector")
        self.GAEngine = None
        self.nMigrationRate = Consts.CDefGenMigrationRate
        self.nIndividuals = Consts.CDefMigrationNIndividuals
        self.nReplacement = Consts.CDefGenMigrationReplacement
        
        self.comm = MPI.COMM_WORLD
        self.pid = self.comm.rank
        
        #now this is fixed
        if self.pid == 0:
            self.source = self.comm.size - 1
        else:
            self.source = self.comm.rank - 1
        self.dest = (self.comm.rank +1) % (self.comm.size)

    def isReady(self):
        """ Returns true if is time to migrate """
        
        if self.GAEngine.getCurrentGeneration() % self.nMigrationRate == 0:
            return True
        else: 
            return False

    def getNumReplacement(self):
        """ Return the number of individuals that will be
        replaced in the migration process """
        return self.nReplacement

    def setNumReplacement(self, num_individuals):
        """ Return the number of individuals that will be
        replaced in the migration process

        :param num_individuals: the number of individuals to be replaced
        """
        self.nReplacement = num_individuals

    def getNumIndividuals(self):
        """ Return the number of individuals that will migrate

        :rtype: the number of individuals to be replaced
        """
        return self.nIndividuals

    def setNumIndividuals(self, num_individuals):
        """ Set the number of individuals that will migrate

        :param num_individuals: the number of individuals
        """
        self.nIndividuals = num_individuals

    def setMigrationRate(self, generations):
        """ Sets the generation frequency supposed to migrate
        and receive individuals.

        :param generations: the number of generations
        """
        self.nMigrationRate = generations

    def getMigrationRate(self):
        """ Return the the generation frequency supposed to migrate
        and receive individuals

        :rtype: the number of generations
        """
        return self.nMigrationRate

    def setGAEngine(self, ga_engine):
        """ Sets the GA Engine handler """
        self.GAEngine = ga_engine

    def start(self):
        """ Initializes the migration scheme """
        pass

    def stop(self):
        """ Stops the migration engine """
        pass

    def getGroupName(self):
        """ Gets the group name

        .. note:: all islands of evolution which are supposed to exchange
                  individuals, must have the same group name.
        """
        return self.groupName

    def setGroupName(self, name):
        """ Sets the group name

        :param name: the group name

        .. note:: all islands of evolution which are supposed to exchange
                  individuals, must have the same group name.
        """
        self.groupName = name

    def select(self):
        """ Pickes an individual from population using specific selection method

        :rtype: an individual object
        """
        if self.selector.isEmpty():
            return self.GAEngine.select(popID=self.GAEngine.currentGeneration)
        else:
            for it in self.selector.applyFunctions(self.GAEngine.internalPop, 
                                                   popID=self.GAEngine.currentGeneration):
                return it

    def selectPool(self, num_individuals):
        """ Select num_individuals number of individuals and return a pool

        :param num_individuals: the number of individuals to select
        :rtype: list with individuals
        """
        pool = [self.select() for i in xrange(num_individuals)]
        return pool

    def exchange(self):
        
        if not self.isReady(): 
            return
        
        pool_to_send = self.selectPool(self.getNumIndividuals())
        self.comm.isend(pool_to_send, dest = self.dest, tag=0)
        
        pool_received = self.comm.recv(source = self.source, tag = 0)
        population = self.GAEngine.getPopulation()

        pool = pool_received
        for i in xrange(self.getNumReplacement()):
            if len(pool) <= 0: 
                break
            choice = rand_choice(pool)
            pool.remove(choice)
        
            # replace the worst
            population[len(population)-1-i] = choice
