//
// Created by cryscan on 8/3/20.
//

#ifndef QUADRUPED_GAITS_H
#define QUADRUPED_GAITS_H

#include <towr/terrain/examples/height_map_examples.h>
#include <towr/nlp_formulation.h>

towr::NlpFormulation gallop();

towr::NlpFormulation trot();

towr::NlpFormulation turn();

towr::NlpFormulation trot_turn();

towr::NlpFormulation idle();

#endif //QUADRUPED_GAITS_H
