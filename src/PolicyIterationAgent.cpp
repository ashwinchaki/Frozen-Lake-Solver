//
// Created by Chi Zhang on 8/26/19.
//

#include "PolicyIterationAgent.hpp"
#include "common.hpp"
#include <cmath>

PolicyIterationAgent::PolicyIterationAgent(const FrozenLakeMDP &mdp, double gamma, int iterations, double threshold) : ValueEstimateAgent(gamma, iterations, threshold), m_mdp(mdp)
{
    MSG("Training Policy Iteration Agent on " << m_mdp.getName());
    MSG("Initializing Policy Iteration Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double PolicyIterationAgent::getQValue(const GameState &state, const Action &action)
{
    // TODO
    // return qValueMap[std::make_pair(state, action)];
    return 0.0;
}

double PolicyIterationAgent::getValue(const GameState &state)
{
    // TODO
    return evaluateCurrentPolicy()[state];
}

Action PolicyIterationAgent::getPolicy(const GameState &state)
{
    // TODO
    return m_policy[state];
}

std::map<GameState, double> PolicyIterationAgent::initializeV()
{
    std::map<GameState, double> val_map;
    std::set<GameState> possStates = m_mdp.getStates();
    std::set<GameState>::iterator it = possStates.begin();
    while (it != possStates.end())
    {
        // for all possible states, initialize value(S) to 0
        GameState state = *it;
        val_map[state] = 0;
        ++it;
    }

    return val_map;
}

/*
 * Evaluate the current policy by returning V(s), which is represented as a map,
 * where key is GameState and value is double.
 */
std::map<GameState, double> PolicyIterationAgent::evaluateCurrentPolicy()
{

    std::map<GameState, double> val_map = initializeV();
    double delta = INT_MAX;
    int iteration = 0;
    while ((delta > m_threshold) && iteration <= m_iterations)
    {
        delta = 0;
        iteration++; // increment iteration

        std::set<GameState> possStates = m_mdp.getStates(); // get all possible states to iterate through
        std::set<GameState>::iterator state_iterator = possStates.begin();

        while (state_iterator != possStates.end())
        {
            // for each possible state in environment
            GameState curr_state = *state_iterator;
            std::cout << "Current State (eval): " << curr_state.getName() << " " << curr_state.getLoc().x << " " << curr_state.getLoc().y << std::endl;

            double temp = val_map[curr_state];

            double newValue = 0;

            if (m_mdp.getPossibleActions(curr_state).size() == 0)
            {
                std::cout << "NO POSSIBLE ACTIONS" << std::endl;

                state_iterator++;
                continue;
            }

            Action action = m_policy[curr_state];

            std::map<GameState, double> transitionProbs = m_mdp.getTransitionStatesAndProbs(curr_state, action);
            std::map<GameState, double>::iterator transition_iterator = transitionProbs.begin();

            if (transitionProbs.size() == 0)
            {
                state_iterator++;
                continue;
            }

            while (transition_iterator != transitionProbs.end())
            {
                std::pair<GameState, double> transition_pair = *transition_iterator;
                std::cout << "TRANSITION STATE (eval): " << transition_pair.first.getLoc().x << " " << transition_pair.first.getLoc().y << " prob: " << transition_pair.second << std::endl;

                transition_iterator++;

                newValue += (transition_pair.second) *
                            (m_mdp.getReward(curr_state, action, transition_pair.first) +
                             (m_gamma * val_map[transition_pair.first]));
            }

            delta = std::max(delta, std::abs(temp - val_map[curr_state]));

            ++state_iterator; // iterate to next state in environment
        }
    }

    return val_map;
}

void PolicyIterationAgent::solve()
{
    // TODO
    bool stable = false;
    int iter = 0;
    while (!stable && (iter++) < m_iterations)
    {
        stable = true;

        std::map<GameState, double> val_map = evaluateCurrentPolicy();

        std::set<GameState> possStates = m_mdp.getStates();
        std::set<GameState>::iterator state_iterator = possStates.begin();

        while (state_iterator != possStates.end()) // for each state s in S
        {
            GameState curr_state = *state_iterator;
            std::cout << "Current State (solve): " << curr_state.getName() << " " << curr_state.getLoc().x << " " << curr_state.getLoc().y << std::endl;
            // std::map<GameState, Action> temp(m_policy); // copy policy map to temp variable
            Action temp = m_policy[curr_state];

            std::vector<Action> possActions = m_mdp.getPossibleActions(curr_state); // get all possible actions
            if (possActions.size() == 0)
            {
                std::cout << "NO POSSIBLE ACTIONS (solve)" << std::endl;
                state_iterator++;
                continue;
            }

            Action optimalAction = DOWN;
            double maxActionValue = INT_MIN;

            for (Action action : possActions)
            {
                std::cout << "WORKING ON ACTION (solve): " << action << std::endl;
                // for each possible action, get all possible transition states and probabilities
                std::map<GameState, double> transitionProbs = m_mdp.getTransitionStatesAndProbs(curr_state, action);
                std::map<GameState, double>::iterator transition_iterator = transitionProbs.begin();

                double sum = 0; // sum for current action

                while (transition_iterator != transitionProbs.end())
                {
                    // for each possible transition state (w/ probability)
                    std::pair<GameState, double> transition_pair = *transition_iterator;
                    std::cout << "TRANSITION STATE (solve): " << transition_pair.first.getLoc().x << " " << transition_pair.first.getLoc().y << " prob: " << transition_pair.second << std::endl;
                    // val = prob(s' | s, a) * [R(s, a, s') + γ(v[s'])]
                    sum += transition_pair.second * (m_mdp.getReward(curr_state, action, transition_pair.first) + (m_gamma * val_map[transition_pair.first]));

                    transition_iterator++; // iterate to next possible transition state
                }

                std::cout << "SUM (solve): " << sum << std::endl;

                // pick optimal action based on current max value
                if (sum > maxActionValue)
                {
                    maxActionValue = sum;
                    optimalAction = action;
                }

                m_policy[curr_state] = optimalAction; // set π[s] = argmax_a
            }

            if (temp != m_policy[curr_state])
                stable = false;

            ++state_iterator; // iterate to next state in environment
        }
    }

    std::cout << "iterations: " << iter << std::endl;
}

void PolicyIterationAgent::initialize()
{
    // TODO
    // valueMap --> initialize to values of 0 for all possible game states?
    std::set<GameState> possStates = m_mdp.getStates();
    std::set<GameState>::iterator it = possStates.begin();
    while (it != possStates.end())
    {
        // for all possible states, initialize value(S) to 0
        GameState state = *it;
        std::vector<Action> possActions = m_mdp.getPossibleActions(state); // get all possible actions
        if (possActions.size() == 0)
            m_policy[state] = LEFT;
        else
            m_policy[state] = possActions[0];
        ++it;
    }
}
