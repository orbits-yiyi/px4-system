/**
 * Teaching module publication interval
 *
 * The module applies a defensive 20..2000 ms limit at runtime as well.
 * The parameter is sampled by the module on its next scheduled execution.
 *
 * @unit ms
 * @min 20
 * @max 2000
 * @group Learning
 */
PARAM_DEFINE_INT32(LEARN_PERIOD, 100);
