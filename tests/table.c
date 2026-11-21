#include "check.h"

static const RantMathFunction *named(const char *name){
    uint32_t i;
    for (i = 0; i < rant_math_function_count(); i++)
        if (strcmp(rant_math_function_at(i)->name, name) == 0) return rant_math_function_at(i);
    return NULL;
}

void test_table(void){
    uint32_t i, j, n = rant_math_function_count(), bad = 0;
    const RantMathFunction *inverse = named("pose_inverse");
    CHECK(n > 400 && rant_math_function_at(n) == NULL, "every call is listed, and none past the end");
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++)
            if (strcmp(rant_math_function_at(i)->name, rant_math_function_at(j)->name) == 0) bad++;
    CHECK(bad == 0, "each call once");
    CHECK(inverse && inverse->input_count == 1 && strcmp(inverse->inputs[0].type, "Pose") == 0 &&
          strcmp(inverse->outputs[0].kind, "return") == 0 && strcmp(inverse->errors, "none") == 0, "a call's inputs and outputs");
    CHECK(named("rant_math_function_count") == NULL && named("math_function_count") == NULL, "the table leaves itself out");
    for (i = 0, bad = 0; i < rant_math_method_count(); i++){
        const RantMathMethod *m = rant_math_method_at(i);
        const RantMathFunction *f = rant_math_function_at(m->function);
        if (!f || (m->self_index != UINT32_MAX && m->self_index >= f->input_count)) bad++;
    }
    CHECK(rant_math_method_count() > 400 && bad == 0, "each method names a call and an input of it");
    for (i = 0, bad = 0; i < rant_math_operator_count(); i++){
        const RantMathOperator *o = rant_math_operator_at(i);
        if (!rant_math_function_at(o->function)) bad++;
        if (strcmp(o->op, "*") == 0 && strcmp(o->left, "Pose") == 0 && strcmp(o->right, "Double3") == 0)
            CHECK(strcmp(rant_math_function_at(o->function)->name, "pose_apply") == 0, "pose times a point applies it");
    }
    CHECK(rant_math_operator_count() > 20 && bad == 0, "each operator names a call");
}
