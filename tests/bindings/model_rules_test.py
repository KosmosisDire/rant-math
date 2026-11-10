"""Each parameter rule of the model on a hand made parameter list, and the order of the rules."""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools", "bindgen"))
import model  # noqa: E402
from model import CParam, Func, Walk  # noqa: E402


def P(name, type, pointer=0, const=False, dim=0, **tags):
    return CParam(name, type, const, pointer, dim, [(k, v) for k, v in tags.items()])


def walk(*params):
    return Walk(Func("rant_x", "x", "x.h", "void", list(params)))


def run(rule, *params):
    w = walk(*params)
    return rule(w), w


def kinds(items):
    return [i.kind for i in items]


def test_declines():
    # Every rule says no to a parameter it does not match, and applies nothing.
    for rule, params in [
        (model.fixed_out, [P("a", "double")]),
        (model.matrix_out, [P("m", "RantMatrixView")]),
        (model.list_counted, [P("a", "double", 1, True), P("b", "double", 1, True)]),
        (model.sized_out, [P("a", "double", 1)]),
        (model.plain_out, [P("a", "double")]),
        (model.count_out, [P("n", "uint32_t", 1)]),
    ]:
        used, w = run(rule, *params)
        assert used is None, rule.__name__
        assert not (w.f.inputs or w.f.outputs or w.f.args), rule.__name__


def test_fixed_out():
    used, w = run(model.fixed_out, P("out", "double", dim=4))
    assert used == 1 and w.f.outputs[0].kind == "fixed" and w.f.outputs[0].size == 4


def test_matrix():
    used, w = run(model.matrix_out, P("m", "RantMatrixView", out="3 4"))
    assert used == 1 and (w.f.outputs[0].rows, w.f.outputs[0].cols) == ("3", "4")
    used, w = run(model.matrix_in, P("m", "RantMatrixView"))
    assert used == 1 and w.f.inputs[0].kind == "matrix" and "m" in w.by_c


def test_list_view_renames_the_input():
    used, w = run(model.list_view, P("poly", "RantPolygonView"))
    assert used == 1 and w.f.inputs[0].name == "points" and w.f.args[0].b == "RantPolygonView"


def test_list_counted_takes_two():
    used, w = run(model.list_counted, P("pts", "RantDouble3", 1, True), P("count", "uint32_t"))
    assert used == 2 and kinds(w.f.args) == ["list_ptr", "list_count"]


def test_one_stops_a_counted_list():
    used, _ = run(model.list_counted, P("a", "double", 1, True, one=""), P("n", "uint32_t"))
    assert used is None


def test_list_degree():
    used, w = run(model.list_degree, P("c", "double", 1, True), P("degree", "uint32_t", degree=""))
    assert used == 2 and kinds(w.f.args) == ["list_ptr", "degree"]


def test_list_with_same_out_takes_three():
    used, w = run(model.list_with_same_out, P("in", "RantDouble3", 1, True), P("out", "RantDouble3", 1),
                  P("n", "uint32_t"))
    assert used == 3 and w.f.outputs[0].kind == "same" and w.f.outputs[0].size is w.f.inputs[0]


def test_list_pair_takes_three():
    used, w = run(model.list_pair, P("a", "double", 1, True), P("b", "double", 1, True), P("n", "uint32_t"))
    assert used == 3 and [i.kind for i in w.f.inputs] == ["list", "list"] and w.f.args[2].b is w.f.inputs[1]


def test_ref_optional():
    used, w = run(model.ref_in, P("a", "RantPose", 1, True, optional=""))
    assert used == 1 and w.f.inputs[0].kind == "opt_ref"
    used, w = run(model.ref_in, P("a", "RantPose", 1, True))
    assert w.f.inputs[0].kind == "ref"


def test_handle_is_not_a_vector():
    used, w = run(model.handle_out, P("tree", "RantKdTree", 2))
    assert used == 1 and w.f.outputs[0].kind == "handle" and not w.f.vector


def test_image_new_reads_its_size_words():
    used, w = run(model.image_new, P("dst", "RantImageView", 1, image="width=w height=h"))
    out = w.f.outputs[0]
    assert used == 1 and (out.cols, out.rows, out.size) == ("w", "h", "src.format")


def test_mesh_new_takes_four():
    used, _ = run(model.mesh_new, P("v", "RantMeshView", 1, mesh=""))
    assert used is None
    used, w = run(model.mesh_new, P("v", "RantImageView", 1, mesh=""))
    assert used == 4 and w.f.outputs[0].kind == "mesh_new"


def test_view_mutated():
    used, w = run(model.view_mutated, P("img", "RantImageView", 1))
    assert used == 1 and w.f.inputs[0].kind == "image_mut" and w.f.outputs[0].size is w.f.inputs[0]


def test_room_then_count():
    w = walk(P("out", "double", 1, room="n"), P("n", "uint32_t", 1))
    assert model.room_out(w) == 1
    w.i = 1
    assert model.count_out(w) == 1
    assert kinds(w.f.args) == ["out", "count_ptr"] and w.f.inputs[0].name == "n" and w.pending == []


def test_query_shares_the_count_of_an_input():
    w = walk(P("pts", "double", 1, True), P("n", "uint32_t"), P("hits", "uint32_t", 1, query="pts"))
    w.by_c["pts"] = "input"
    w.i = 2
    assert model.query_out(w) == 1 and w.f.outputs[0].size == "input" and len(w.pending) == 1


def test_query_with_count_takes_two():
    used, w = run(model.query_with_count, P("out", "double", 1, per="3"), P("n", "uint32_t", 1))
    assert used == 2 and w.f.outputs[0].per == 3 and kinds(w.f.args) == ["out", "count_ptr"]


def test_sized_out_takes_two():
    used, w = run(model.sized_out, P("out", "double", 1), P("count", "uint32_t"))
    assert used == 2 and w.f.outputs[0].kind == "sized" and w.f.inputs[0].name == "count"


def test_size_out_and_same_out():
    used, w = run(model.size_out, P("out", "double", 1, size="n"))
    assert used == 1 and w.f.outputs[0].size == "n"
    w = walk(P("pts", "double", 1, True), P("out", "double", 1, same="pts"))
    w.by_c["pts"] = "input"
    w.i = 1
    assert model.same_out(w) == 1 and w.f.outputs[0].size == "input"


def test_order_decides():
    # A uint32_t pointer is a count after a room, a plain out alone, and a sized out never takes it.
    f = Func("rant_x", "x", "x.h", "void", [P("out", "double", 1, room="n"), P("n", "uint32_t", 1)])
    model.auto(f, None)
    assert kinds(f.outputs) == ["room"] and kinds(f.args) == ["out", "count_ptr"]
    f = Func("rant_x", "x", "x.h", "void", [P("n", "uint32_t", 1)])
    model.auto(f, None)
    assert kinds(f.outputs) == ["out"]


def test_return_after_the_outs():
    f = Func("rant_x", "x", "x.h", "double", [P("a", "double"), P("out", "double", 1)], tags=[("last", "dist")])
    model.auto(f, None)
    assert [o.name for o in f.outputs] == ["out", "dist"]


def test_nothing_to_return_is_an_error():
    try:
        model.auto(Func("rant_x", "x", "x.h", "void", [P("a", "double")]), None)
    except ValueError:
        return
    raise AssertionError("no error")


if __name__ == "__main__":
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    for name, fn in tests:
        fn()
    print(f"{len(tests)} rule tests passed")
