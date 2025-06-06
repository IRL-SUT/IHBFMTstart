#ifndef OMPL_GEOMETRIC_PLANNERS_BIDIRECTIONALFMT_H
#define OMPL_GEOMETRIC_PLANNERS_BIDIRECTIONALFMT_H

#include <ompl/geometric/planners/PlannerIncludes.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/datastructures/BinaryHeap.h>
#include <ompl/base/OptimizationObjective.h>
#include <map>
#include <utility>

// modify
#include <fstream>
#include <stdio.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <ompl/base/samplers/InformedStateSampler.h>

#include <execinfo.h>
#include <unistd.h>
#include <unordered_set>

inline void printBacktrace()
{
    void *array[20];
    size_t size = backtrace(array, 20);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
}


using namespace cv; //OPENCV
namespace ompl
{
    namespace geometric
    {
        class IHBFMT : public ompl::base::Planner
        {
        public:
            /** \brief Tree identifier */
            enum TreeType
            {
                FWD = 0,
                REV = 1
            };

            /** \brief Exploration strategy identifier */
            enum ExploreType
            {
                SWAP_EVERY_TIME = 0,
                CHOOSE_SMALLEST_Z = 1
            };

            /** \brief Termination strategy identifier */
            enum TerminateType
            {
                FEASIBILITY = 0,
                OPTIMALITY = 1
            };

            IHBFMT(const base::SpaceInformationPtr &si);

            ~IHBFMT() override;

            void setup() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void getPlannerData(base::PlannerData &data) const override;



            void setCostTheshold(double cost)  // cost_threshold
            {
                 costthrold = base::Cost(cost);
            }
            double getCostTheshold()       // cost_threshold
            {
                return costthrold.value();
            }


            void setNumSamples(const unsigned int numSamples)
            {
                numSamples_ = numSamples;
            }

            /** \brief Get the number of states that the planner will sample */
            unsigned int getNumSamples() const
            {
                return numSamples_;
            }

            /** \brief If nearestK is true, FMT will be run using the Knearest strategy */
            void setNearestK(bool nearestK)
            {
                nearestK_ = nearestK;
            }

            /** \brief Get the state of the nearestK strategy */
            bool getNearestK() const
            {
                return nearestK_;
            }

            void setRadiusMultiplier(const double radiusMultiplier)
            {
                if (radiusMultiplier <= 0.0)
                    throw Exception("Radius multiplier must be greater than zero");
                radiusMultiplier_ = radiusMultiplier;
            }

            /** \brief Get the multiplier used for the nearest neighbors search
                radius */
            double getRadiusMultiplier() const
            {
                return radiusMultiplier_;
            }

            /** \brief Store the volume of the obstacle-free configuration space.
                If no value is specified, the default assumes an obstacle-free
                unit hypercube, freeSpaceVolume = (maximumExtent/sqrt(dimension))^(dimension) */
            void setFreeSpaceVolume(const double freeSpaceVolume)
            {
                if (freeSpaceVolume < 0.0)
                    throw Exception("Free space volume should be greater than zero");
                freeSpaceVolume_ = freeSpaceVolume;
            }

            /** \brief Get the volume of the free configuration space that is
                being used by the planner */
            double getFreeSpaceVolume() const
            {
                return freeSpaceVolume_;
            }

            /** \brief Sets the collision check caching to save calls to the collision
                checker with slightly memory usage as a counterpart */
            void setCacheCC(bool ccc)
            {
                cacheCC_ = ccc;
            }

            /** \brief Get the state of the collision check caching */
            bool getCacheCC() const
            {
                return cacheCC_;
            }

            /** \brief Activates the cost to go heuristics when ordering the heap */
            void setHeuristics(bool h)
            {
                heuristics_ = h;
            }

            /** \brief Returns true if the heap is ordered taking into account
                cost to go heuristics */
            bool getHeuristics() const
            {
                return heuristics_;
            }

            /** \brief Activates the extended FMT*: adding new samples if planner does not finish successfully. */
            void setExtendedFMT(bool e)
            {
                extendedFMT_ = e;
            }

            /** \brief Returns true if the extended FMT* is activated. */
            bool getExtendedFMT() const
            {
                return extendedFMT_;
            }

            /** \brief Sets exploration strategy: balanced true expands one tree every iteration.
                 False will select the tree with lowest maximum cost to go. */
            void setExploration(bool balanced)
            {
                exploration_ = SWAP_EVERY_TIME;
                if (balanced)
                {
                    exploration_ = CHOOSE_SMALLEST_Z;
                }
            }

            /** \brief Returns the exploration strategy. */
            bool getExploration() const
            {
                return (exploration_ == CHOOSE_SMALLEST_Z);
            }

            /** \brief Sets the termination strategy: optimality true finishes when the best
                possible path is found. Otherwise, the algorithm will finish when the first
                feasible path is found. */
            void setTermination(bool optimality)
            {
                termination_ = FEASIBILITY;
                if (optimality)
                {
                    termination_ = OPTIMALITY;
                }
            }

            /** \brief Returns the termination strategy. */
            bool getTermination() const
            {
                return (termination_ == OPTIMALITY);
            }

            /** \brief Sets Nearest Neighbors precomputation. Currently, it precomputes
                once solve() has been called. */
            void setPrecomputeNN(bool p)
            {
                precomputeNN_ = p;
            }

            /** \brief Returns true if Nearest Neighbor precomputation is done. */
            bool setPrecomputeNN() const
            {
                return precomputeNN_;
            }

            /** \brief Representation of a bidirectional motion. */
            class BiDirMotion
            {
            public:
                enum SetType
                {
                    SET_CLOSED,
                    SET_OPEN,
                    SET_UNVISITED
                };

                BiDirMotion(TreeType *tree) : state_(nullptr), tree_(tree)//构造函数
                {
                    parent_[FWD] = nullptr;
                    parent_[REV] = nullptr;
                    cost_[FWD] = base::Cost(0.0);
                    cost_[REV] = base::Cost(0.0);
                    hcost_[FWD] = base::Cost(0.0);
                    hcost_[REV] = base::Cost(0.0);
                    currentSet_[FWD] = SET_UNVISITED;
                    currentSet_[REV] = SET_UNVISITED;
                }

                /** \brief Constructor that allocates memory for the state */
                BiDirMotion(const base::SpaceInformationPtr &si, TreeType *tree) : state_(si->allocState()), tree_(tree)
                {
                    parent_[FWD] = nullptr;
                    parent_[REV] = nullptr;

                    cost_[FWD] = base::Cost(std::numeric_limits<double>::infinity());
                    cost_[REV] = base::Cost(std::numeric_limits<double>::infinity());

                    hcost_[FWD] = base::Cost(0.0);
                    hcost_[REV] = base::Cost(0.0);
                    currentSet_[FWD] = SET_UNVISITED;
                    currentSet_[REV] = SET_UNVISITED;
                }

                typedef std::vector<BiDirMotion *> BiDirMotionPtrs;

                /** \brief The state contained by the motion */
                base::State *state_;

                /** \brief The parent motion in the exploration tree  */
                BiDirMotion *parent_[2];

                /** \brief The set of motions descending from the current motion  */
                BiDirMotionPtrs children_[2];

                /** \brief Current set in which the motion is included.  */
                SetType currentSet_[2];

                /** \brief Tree identifier  */
                TreeType *tree_;

                /** \brief The cost of this motion  */
                base::Cost cost_[2];

                /** \brief The minimum cost to go of this motion (heuristically computed) */
                base::Cost hcost_[2];

                /** \brief Contains the connections attempted FROM this node */
                std::set<BiDirMotion *> collChecksDone_;

                /** \brief Set the state associated with the motion */
                inline base::Cost getCost() const
                {
                    return this->cost_[*tree_];
                }

                /** \brief Get cost of this motion in the inactive tree */
                inline base::Cost getOtherCost() const
                {
                    return this->cost_[(*tree_ + 1) % 2];
                }

                /** \brief Set the cost of the motion */
                inline void setCost(base::Cost cost)
                {

                    if (cost.value() < 0)
                    {
                        std::cerr << "ERROR: Attempt to set negative cost: " << cost.value() << std::endl;
                        printBacktrace();
                        abort();
                    }

                    this->cost_[*tree_] = cost;
                }

                /** \brief Set the parent of the motion */
                inline void setParent(BiDirMotion *parent)
                {
                    this->parent_[*tree_] = parent;
                }

                /** \brief Get the parent of the motion */
                inline BiDirMotion *getParent() const
                {
                    return this->parent_[*tree_];
                }

                /** \brief Set the children of the motion */
                inline void setChildren(BiDirMotionPtrs children)
                {
                    this->children_[*tree_] = std::move(children);
                }

                /** \brief Get the children of the motion */
                inline BiDirMotionPtrs &getChildren()
                {
                    return this->children_[*tree_];
                }

                /** \brief Set the current set of the motion */
                inline void setCurrentSet(SetType set)
                {
                    this->currentSet_[*tree_] = set;
                }

                /** \brief Fet the current set of the motion */
                inline SetType getCurrentSet() const
                {
                    return this->currentSet_[*tree_];
                }

                /** \brief Get set of this motion in the inactive tree */
                inline SetType getOtherSet() const
                {
                    return this->currentSet_[(*tree_ + 1) % 2];
                }

                /** \brief Set tree identifier for this motion */
                inline void setTreeType(TreeType *treePtr)
                {
                    this->tree_ = treePtr;
                }

                /** \brief Get tree identifier for this motion */
                inline TreeType getTreeType() const
                {
                    return *tree_;
                }

                /** \brief Set the state associated with the motion */
                void setState(base::State *state)
                {
                    state_ = state;
                }

                /** \brief Get the state associated with the motion */
                base::State *getState() const
                {
                    return state_;
                }

                /** \brief Returns true if the connection to m has been already
                    tested and failed because of a collision */
                bool alreadyCC(BiDirMotion *m)
                {
                    return !(collChecksDone_.find(m) == collChecksDone_.end());
                }

                /** \brief Caches a failed collision check to m */
                void addCC(BiDirMotion *m)
                {
                    collChecksDone_.insert(m);
                }

                /** \brief Set the cost to go heuristic cost */
                void setHeuristicCost(const base::Cost h)
                {
                    hcost_[*tree_] = h;
                }

                /** \brief Get the cost to go heuristic cost */
                base::Cost getHeuristicCost() const
                {
                    return hcost_[*tree_];
                }
            };

            typedef std::vector<BiDirMotion *> BiDirMotionPtrs;

        protected:
            /** \brief Comparator used to order motions in a binary heap */
            struct BiDirMotionCompare
            {
                bool operator()(const BiDirMotion *p1, const BiDirMotion *p2) const
                {
                    if (heuristics_)
                        return (opt_->combineCosts(p1->getCost(), p1->getHeuristicCost()).value() <
                                opt_->combineCosts(p2->getCost(), p2->getHeuristicCost()).value());
                    return (p1->getCost().value() < p2->getCost().value());
                }

                base::OptimizationObjective *opt_;
                bool heuristics_;
            };

            typedef ompl::BinaryHeap<BiDirMotion *, BiDirMotionCompare> BiDirMotionBinHeap;

            /** \brief Change the active tree */
            void swapTrees();

            /** \brief Sets forward tree active */
            void useFwdTree()
            {
                tree_ = FWD;
            }

            /** \brief Sets reverse tree active */
            void useRevTree()
            {
                tree_ = REV;
            }

            /** \brief Compute the distance between two motions as the cost
                between their contained states. Note that for computationally
                intensive cost functions, the cost between motions should be
                stored to avoid duplicate calculations */
            double distanceFunction(const BiDirMotion *a, const BiDirMotion *b) const
            {
                return opt_->motionCost(a->getState(), b->getState()).value();
            }

            /** \brief Compute the volume of the unit ball in a given dimension */
            double calculateUnitBallVolume(unsigned int dimension) const;

            /** \brief Calculate the radius to use for nearest neighbor searches,
                 using the bound given in [L. Janson, E. Schmerling, A. Clark, M. Pavone. Fast marching tree: a fast
               marching sampling-based method for optimal motion planning in many dimensions. The International Journal
               of Robotics Research, 34(7):883-921, 2015](http://arxiv.org/pdf/1306.3532.pdf). The radius depends on
                 the radiusMultiplier parameter, the volume of the free
                 configuration space, the volume of the unit ball in the current
                 dimension, and the number of nodes in the graph */
            double calculateRadius(unsigned int dimension, unsigned int n) const;

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Save the neighbors within a neighborhood of a given state. The strategy
                used (nearestK or nearestR depends on the planner configuration */
            void saveNeighborhood(const std::shared_ptr<NearestNeighbors<BiDirMotion *>> &nn, BiDirMotion *m);

            /** \brief Sample a state from the free configuration space and save
                it into the nearest neighbors data structure */
            void sampleFree(const std::shared_ptr<NearestNeighbors<BiDirMotion *>> &nn,
                            const base::PlannerTerminationCondition &ptc);

            /** \brief Carries out some planner checks */
            void initializeProblem(base::GoalSampleableRegion *&goal_s);

            /** \brief Complete one iteration of the main loop of the IHBFMT* algorithm:
                Find K nearest nodes in set Unvisited (or within a radius r) of the node z.
                Attempt to connect them to their optimal cost-to-come parent
                in set Open. Remove all newly connected nodes fromUnvisited and insert
                them into Open. Remove motion z from Open, and update z to be the
                current lowest cost-to-come node in Open */
            void expandTreeFromNode(BiDirMotion *&z, BiDirMotion *&connection_point);

            /** \brief Executes the actual planning algorithm, swapping and expanding the trees */
            bool plan(BiDirMotion *x_init, BiDirMotion *x_goal, BiDirMotion *&connection_point,
                      const base::PlannerTerminationCondition &ptc);

            /** \brief Checks if the termination condition is met */
            bool termination(BiDirMotion *&z, BiDirMotion *&connection_point,
                             const base::PlannerTerminationCondition &ptc);

            /** \brief Chooses and expand a tree according to the exploration strategy */
            void chooseTreeAndExpansionNode(BiDirMotion *&z);

            /** \brief Trace the path along a tree towards the root (forward or reverse) */
            void tracePath(BiDirMotion *z, BiDirMotionPtrs &path);

            /** \brief For a motion m, updates the stored neighborhoods of all its neighbors by
                by inserting m (maintaining the cost-based sorting) */
            void updateNeighborhood(BiDirMotion *m, std::vector<BiDirMotion *> nbh);

            /** \brief Extended FMT strategy: inserts a new motion in open if the heap is empty */
            void insertNewSampleInOpen(const base::PlannerTerminationCondition &ptc);

            /** \brief The number of samples to use when planning */
            unsigned int numSamples_{750u};

            /** \brief This planner uses a nearest neighbor search radius
                proportional to the lower bound for optimality derived for FMT*
                in Section 4 of [L. Janson, E. Schmerling, A. Clark, M. Pavone. Fast marching tree: a fast marching
               sampling-based method for optimal motion planning in many dimensions. The International Journal of
               Robotics Research, 34(7):883-921, 2015](http://arxiv.org/pdf/1306.3532.pdf).  The radius multiplier
                is the multiplier for the lower bound. For guaranteed asymptotic
                convergence, the user should choose a multiplier for the search
                radius that is greater than one. The default value is 1.1.
                In general, a radius between 0.9 and 5 appears to perform the best */
            double radiusMultiplier_{1.};

            /** \brief The volume of numSathe free configuration space, computed
                as an upper bound with 95% confidence */
            double freeSpaceVolume_;

            /** \brief Number of collision checks performed by the algorithm */
            unsigned int collisionChecks_{0u};

            /** \brief Flag to activate the K nearest neighbors strategy */
            bool nearestK_{true};

            /** \brief Radius employed in the nearestR strategy. */
            double NNr_{0.};

            /** \brief K used in the nearestK strategy */
            unsigned int NNk_{0};

            /** \brief Active tree */
            TreeType tree_{FWD};

            /** \brief Exploration strategy used */
            ExploreType exploration_{SWAP_EVERY_TIME};

            /** \brief Termination strategy used */
            TerminateType termination_{OPTIMALITY};

            /** \brief If true all the nearest neighbors maps are precomputed before solving. */
            bool precomputeNN_{true}; //(llx)

            /** \brief A nearest-neighbor datastructure containing the set of all motions */
            std::shared_ptr<NearestNeighbors<BiDirMotion *>> nn_;

            /** \brief A map linking a motion to all of the motions within a
                distance r of that motion */
            std::map<BiDirMotion *, BiDirMotionPtrs> neighborhoods_;

            /** \brief A binary heap for storing explored motions in
                cost-to-come sorted order. The motions in Open have been explored,
                yet are still close enough to the frontier of the explored set Open
                to be connected to nodes in the unexplored set Unvisited */
            BiDirMotionBinHeap Open_[2];

            /** \brief Map to know the corresponding heap element from the given motion */
            std::map<BiDirMotion *, BiDirMotionBinHeap::Element *> Open_elements[2];

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief The cost objective function */
            base::OptimizationObjectivePtr opt_;

            /** \brief Flag to activate the cost to go heuristics */
            bool heuristics_{true};

            /** \brief Goal state caching to accelerate cost to go heuristic computation */
            base::State *heurGoalState_[2];

            /** \brief Flag to activate the collision check caching */
            bool cacheCC_{true};

            /** \brief Add new samples if the tree was not able to find a solution. */
            bool extendedFMT_{true};
            base::Cost tempCost;
            base::Cost previousCost;
            base::Cost lastCost;
            base::Cost costthrold;
            BiDirMotion *initMotion;
            BiDirMotion *goalMotion;
            base::State *goalState_;
            BiDirMotion *connect_motion_=nullptr;
            bool successful_{false};
            bool connectchange_{false};
            bool firstSuccessful_{false};


            void traceSolutionPathThroughTree(BiDirMotion *&connection_point);
            void drawPathe();
            void improvedExpandTreeFromNode(BiDirMotion *&z, BiDirMotion *&connection_point);
            void updateChildCosts(BiDirMotion *m, std::unordered_set<BiDirMotion*> &visited);
            void removeFromParent(BiDirMotion *m);
            int mPathSize_;
            BiDirMotionPtrs mpath_;
        private:
            bool displayInit_{false};
            const double DISP_SCALE_ = 0.65;

            void initDisplayOnce()
            {
            if (!displayInit_)
            {
                cv::namedWindow("path", cv::WINDOW_NORMAL);
                displayInit_ = true;
            }
            }

            /*original image variable*/
            /*you should set your own map path*/
            Mat envImage_ = cv::imread("/home/jing/workspace/IHBFMTstart-main/ompl-1.3.1-2d-IHBFMTstart-beta/tests/resources/ppm/floor.ppm", 1);

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost> &costs, const base::OptimizationObjective &opt)
                  : costs_(costs), opt_(opt)
                {
                }
                bool operator()(unsigned i, unsigned j)
                {
                    return (costs_[i].value() < costs_[j].value());
                }
                const std::vector<base::Cost> &costs_;
                const base::OptimizationObjective &opt_;
            };
        };

    }  // End "geometric" namespace
}  // End "ompl" namespace

#endif /* OMPL_GEOMETRIC_PLANNERS_BIDIRECTIONALFMT_H */

