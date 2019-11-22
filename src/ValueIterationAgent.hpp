//
// Created by Chi Zhang on 8/24/19.
//

#ifndef FROZEN_LAKE_VALUEITERATIONAGENT_HPP
#define FROZEN_LAKE_VALUEITERATIONAGENT_HPP


#include "LearningAgent.hpp"
#include "FrozenLake.hpp"

class ValueIterationAgent : public ValueEstimateAgent {
public:

    ValueIterationAgent(FrozenLakeMDP const &mdp, double gamma, int iterations, double threshold);

    double getValue(const GameState &state) override;

    double getQValue(const GameState &state, const Action &action) override;

    Action getPolicy(const GameState &state) override;

    std::string getName() const override {
        return "ValueIterationAgent";
    }

private:
    void initialize() override;

    void solve();

    const FrozenLakeMDP &m_mdp;

};


#endif //FROZEN_LAKE_VALUEITERATIONAGENT_HPP
