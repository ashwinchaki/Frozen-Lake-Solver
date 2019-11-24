//
// Created by Chi Zhang on 8/24/19.
//

#include "common.hpp"
#include "ValueIterationAgent.hpp"

ValueIterationAgent::ValueIterationAgent(FrozenLakeMDP const &mdp, double gamma, int iterations, double threshold) : ValueEstimateAgent(gamma, iterations, threshold), m_mdp(mdp)
{
    MSG("Training Value Iteration Agent on " << m_mdp.getName());
    MSG("Initializing Value Iteration Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double ValueIterationAgent::getValue(const GameState &state)
{
    // TODO
    return 0.0;
}

double ValueIterationAgent::getQValue(const GameState &state, const Action &action)
{
    // TODO
    return 0.0;
}

Action ValueIterationAgent::getPolicy(const GameState &state)
{
    // TODO
    return LEFT;
}

void ValueIterationAgent::solve()
{
    // TODO. Implement Value Iteration here
    double delta = 0;

    while (!(delta < m_threshold))
    {
        delta = 0; // reset delta value to 0

        std::set<GameState> possStates = m_mdp.getStates(); // get all possible states to iterate through
        std::set<GameState>::iterator state_iterator = possStates.begin();
        while (state_iterator != possStates.end())
        {
            // for each possible state in environment
            GameState curr_state = *state_iterator;
            double temp = (*valueMap.find(curr_state)).second;
            double v = -1;

            std::vector<Action> possActions = m_mdp.getPossibleActions(curr_state); // get all possible actions

            for (Action action : possActions)
            {
                // for each possible action, get all possible transition states and probabilities
                std::map<GameState, double> transitionProbs = m_mdp.getTransitionStatesAndProbs(curr_state, action);
                std::map<GameState, double>::iterator transition_iterator = transitionProbs.begin();

                double sum = 0;

                while (transition_iterator != transitionProbs.end())
                {
                    // for each possible transition state (w/ probability)
                    std::pair<GameState, double> transition_pair = *transition_iterator;
                    // val = prob(s' | s, a) * [R(s, a, s') + γ(v[s'])]
                    sum += transition_pair.second * (m_mdp.getReward(curr_state, action, transition_pair.first) + (m_gamma * (*valueMap.find(transition_pair.first)).second));

                    transition_iterator++; // iterate to next possible transition state
                }
                // v value is larger of current sum (for current action) or current v value
                v = std::max(sum, v);
            }

            (*valueMap.find(curr_state)).second = v; // set value for V[s] to newly calculated v value

            delta = std::max(delta, std::abs(temp - v));

            state_iterator++; // iterate to next state in environment
        }
    }
}

void ValueIterationAgent::initialize()
{
    // TODO. Initialize your data structure.

    // valueMap --> initialize to values of 0 for all possible game states?
    std::set<GameState> possStates = m_mdp.getStates();
    std::set<GameState>::iterator it = possStates.begin();
    while (it != possStates.end())
    {
        // for all possible states, initialize value(S) to 0
        GameState state = *it;
        valueMap.insert(std::pair<GameState, double>(state, 0));
        it++;
    }
}
