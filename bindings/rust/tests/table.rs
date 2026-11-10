//! The library's table of itself, read through the crate.

use rant_math::table;

#[test]
fn the_table_reads() {
    let functions = table::functions();
    let inverse = functions.iter().position(|f| f.name == "pose_inverse").expect("pose_inverse is listed");
    assert_eq!(functions[inverse].inputs[0].type_name, "Pose");
    assert_eq!(functions[inverse].outputs[0].kind, "return");
    let method = table::methods().into_iter().find(|m| m.type_name == "Pose" && m.name == "inverse").unwrap();
    assert_eq!(method.function, inverse);
    assert_eq!(method.self_index, Some(0));
    assert!(table::methods().iter().any(|m| m.type_name == "Quaternion" && m.name == "from_axis_angle" && m.self_index.is_none()));
    let times = table::operators().into_iter().find(|o| o.op == "*" && o.left == "Pose" && o.right == "Double3").unwrap();
    assert_eq!(functions[times.function].name, "pose_apply");
}

#[cfg(feature = "quickjs")]
#[test]
fn names_come_from_the_table() {
    let names = rant_math::quickjs::names();
    assert!(names.contains("export const NAMES = [\n"));
    assert!(names.contains(r#"{"name": "poseInverse", "takes": ["p"], "words": "pose inverse"}"#));
}
