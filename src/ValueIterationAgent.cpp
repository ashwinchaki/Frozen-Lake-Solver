//
// Created by Chi Zhang on 8/24/19.
//

#include "common.hpp"
#include "ValueIterationAgent.hpp"

ValueIterationAgent::ValueIterationAgent(FrozenLakeMDP const &mdp, double gamma, int iterations, double threshold) :
        ValueEstimateAgent(gamma, iterations, threshold), m_mdp(mdp) {
    MSG("Training Value Iteration Agent on " << m_mdp.getName());
    MSG("Initializing Value Iteration Agent");
    initialize();
    MSG("Solving...");
    solve();
}

double ValueIterationAgent::getValue(const GameState &state) {
    // TODO
    return 0.0;
}

double ValueIterationAgent::getQValue(const GameState &state, const Action &action) {
    // TODO
    return 0.0;
}

Action ValueIterationAgent::getPolicy(const GameState &state) {
    // TODO
    return LEFT;
}


void ValueIterationAgent::solve() {
    // TODO. Implement Value Iteration here
}

void ValueIterationAgent::initialize() {
    // TODO. Initialize your data structure.
}




