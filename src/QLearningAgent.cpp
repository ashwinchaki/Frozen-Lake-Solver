//
// Created by Chi Zhang on 8/27/19.
//

#include "common.hpp"
#include "third_party/random.hpp"
#include "QLearningAgent.hpp"
#include <iostream>
#include <fstream>
#include <queue>

QLearningAgent::QLearningAgent(FrozenLakeEnv &env, double gamma, int iterations, double alpha, double epsilon) : ValueEstimateAgent(gamma, iterations, 0.0), m_alpha(alpha), m_epsilon(epsilon), m_env(env)
{
    MSG("Training Q Learning Agent on " << m_env.getName());
    MSG("Initializing Q Learning Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double QLearningAgent::getValue(const GameState &state)
{
    // TODO
    return 0.0;
}

double QLearningAgent::getQValue(const GameState &state, const Action &action)
{
    // TODO
    return m_qvalue[std::make_pair(state, action)];
}

// The final policy without exploration. Used for evaluation.
Action QLearningAgent::getPolicy(const GameState &state)
{
    // TODO
    return m_policy[state];
}

// you should use getAction in solve instead of getPolicy and implement your exploration strategy here.
Action QLearningAgent::getAction(const GameState &state)
{
    // TODO implement exploration

    // TODO: implement random action selection

    std::default_random_engine rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist_double(0.00, 1.00);

    std::vector<Action> possActions = m_env.getPossibleActions(state);
    std::uniform_int_distribution<> dist_int(0, (int)(possActions.size() - 1));

    double probability = dist_double(gen);
    std::cout << "probability of random: " << probability << std::endl;

    // check if m_policy[state] does not exist (i.e. unexplored state)
    Action action = possActions[dist_int(gen)];

    if (m_policy.find(state) == m_policy.end())
    {
        // if state is not in policy map
        return action;
    }

    if (probability < m_epsilon)
    {
        // give random action
        // if action is current policy, generate new action
        while (action == m_policy[state])
            action = possActions[dist_int(gen)];

        return action;
    }

    else // if probability > m_epsilon, pick current best option (m_policy)
    {
        return m_policy[state];
    }

    return LEFT; // control will never reach this point
}

void QLearningAgent::update(const GameState &state, const Action &action, const GameState &nextState, double reward)
{
    // TODO

    if (m_qvalue.find(std::make_pair(state, action)) == m_qvalue.end())
    {
        // * IF Q[S,A] DOES NOT EXIST
        // * SUBSTITUTE 0
        m_qvalue[std::make_pair(state, action)] = 0;
    }

    // * IF π[S] DOES NOT EXIST, INITIALIZE WITH CURRENT ACTION
    if (m_policy.find(state) == m_policy.end())
        m_policy[state] = action;

    // * IF π[S'] DOES NOT EXIST, INITIALIZE TO FIRST POSSIBLE ACTION (OR 0 FOR NO POSSIBLE ACTIONS)
    // * AND INITIALIZE Q[S', π[S']] = 0 AND USE THAT VALUE
    if (m_policy.find(nextState) == m_policy.end())
    {
        std::vector<Action> possActions = m_env.getPossibleActions(nextState);
        if (possActions.size() == 0)
            m_policy[nextState] = LEFT;
        else
            m_policy[nextState] = possActions[0];
    }

    // * INITIALIZE Q[S', π[S']] TO 0 IF IT DOES NOT EXIST
    if (m_qvalue.find(std::make_pair(nextState, m_policy[nextState])) == m_qvalue.end())
        m_qvalue[std::make_pair(nextState, m_policy[nextState])] = 0;

    // ! Q[s, a] = Q[s, a] + alpha[reward + gamma(max_a'(Q[s', a'])) - Q[s, a]]
    // * IF BOTH Q[S,A] and Q[S', A'] EXIST
    m_qvalue[std::make_pair(state, action)] = m_qvalue[std::make_pair(state, action)] + m_alpha * (reward + m_gamma * (m_qvalue[std::make_pair(state, m_policy[nextState])]) - m_qvalue[std::make_pair(state, action)]);

    // * UPDATE π[STATE] TO MAXa Q[S,A]

    Action maxAction = m_policy[state];
    double maxQ = INT_MIN;

    std::vector<Action> possActions = m_env.getPossibleActions(state);
    for (Action poss : possActions)
    {
        if (m_qvalue.find(std::make_pair(state, poss)) == m_qvalue.end())
        {
            // if action not in map, continue;
            continue;
        }
        if (m_qvalue[std::make_pair(state, poss)] > maxQ)
        {
            maxQ = m_qvalue[std::make_pair(state, poss)];
            maxAction = poss;
        }
    }

    m_policy[state] = maxAction; // update policy map?

    return;
}

void QLearningAgent::solve()
{
    // output a file for plotting
    std::ofstream outFile;
    outFile.open("result.csv");
    outFile << "Episode,Reward" << std::endl;

    int maxEpisodeSteps = 100;
    // collect m_iterations trajectories for update
    for (int i = 0; i < m_iterations; i++)
    {
        int numSteps = 0;
        GameState state = m_env.reset(); // reset environment and get initial state
        while (!m_env.isTerminal(state))
        {
            Action action = getAction(state); // use instead of getPolicy?
            GameState nextState = m_env.getNextState(state, action);
            double reward = m_env.getReward(state, action, nextState);

            update(state, action, nextState, reward);
            state = nextState;
            numSteps += 1;
            if (numSteps >= maxEpisodeSteps)
                break; // avoid infinite loop in some cases.
        }
        // evaluate for 100 episodes using the current optimal policy. You can't change this line.
        double episodeReward = m_env.runGame(*this, 100, m_gamma, false).first;
        std::cout << "Evaluating episode reward at learning iteration " << i << " is " << episodeReward << std::endl;
        outFile << i << "," << episodeReward << std::endl;
    }
    outFile.close();
}

void QLearningAgent::initialize()
{
    // TODO initialize m_policy and m_qvalue maps
    /* // // * implement DFS/BFS to initialize map to all possible states?
    // std::set<GameState> visitedStates;  // set of all currently visited states
    // std::queue<GameState> queuedStates; // queued states to search

    // // std::vector<Action> possibleActionList = {LEFT, RIGHT, UP, DOWN};

    // GameState state = m_env.reset();
    // visitedStates.insert(state);
    // queuedStates.push(state);

    // while (!queuedStates.empty())
    // {
    //     // get current working state
    //     GameState curr_state = queuedStates.front();
    //     queuedStates.pop();
    //     visitedStates.insert(curr_state);

    //     // get all possible actions for current state
    //     std::vector<Action> possActions = m_env.getPossibleActions(curr_state);
    //     for (Action action : possActions)
    //     {
    //         // for each action, insert into map w/ current gamestate
    //         m_qvalue[std::make_pair(curr_state, action)] = 0;

    //         // get the resulting state and if not already in queue, add it
    //         GameState transitionState = m_env.getNextState(curr_state, action);
    //         if (visitedStates.find(transitionState) == visitedStates.end())
    //             queuedStates.push(transitionState);
    //     }
    // }

    // // * TEST INITIALIZATION CODE?
    // std::map<std::pair<GameState, Action>, double>::iterator qValueIterator = m_qvalue.begin();
    // while (qValueIterator != m_qvalue.end())
    // {
    //     std::cout << "x: " << (*qValueIterator).first.first.getLoc().x << " y: " << (*qValueIterator).first.first.getLoc().y
    //               << " action: " << (*qValueIterator).second << " qValue: " << (*qValueIterator).second << std::endl;

    //     qValueIterator++;
    // } */

    return;
}
