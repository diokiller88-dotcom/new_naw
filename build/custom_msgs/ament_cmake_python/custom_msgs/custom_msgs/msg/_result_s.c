// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from custom_msgs:msg/Result.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "custom_msgs/msg/detail/result__struct.h"
#include "custom_msgs/msg/detail/result__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool custom_msgs__msg__result__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[31];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("custom_msgs.msg._result.Result", full_classname_dest, 30) == 0);
  }
  custom_msgs__msg__Result * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // is_valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // res_pose_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "res_pose_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->res_pose_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // res_pose_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "res_pose_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->res_pose_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // res_vel_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "res_vel_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->res_vel_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // res_vel_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "res_vel_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->res_vel_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // yaw_diff
    PyObject * field = PyObject_GetAttrString(_pymsg, "yaw_diff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->yaw_diff = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vyaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "vyaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vyaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * custom_msgs__msg__result__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Result */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("custom_msgs.msg._result");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Result");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  custom_msgs__msg__Result * ros_message = (custom_msgs__msg__Result *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // res_pose_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->res_pose_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "res_pose_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // res_pose_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->res_pose_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "res_pose_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // res_vel_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->res_vel_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "res_vel_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // res_vel_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->res_vel_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "res_vel_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // yaw_diff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->yaw_diff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "yaw_diff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vyaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vyaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vyaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
