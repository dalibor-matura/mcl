#include <boost/test/unit_test_suite.hpp>
#include <boost/test/parameterized_test.hpp>
#include <boost/typeof/typeof.hpp>

#include <type_traits>

#define BOOST_FIXTURE_PARAM_TEST_CASE( test_name, F, mbegin, mend )     \
struct test_name : public F {                                           \
	typedef ::std::remove_const< ::std::remove_reference< BOOST_TYPEOF(*(mbegin)) >::type>::type param_t; \
	void test_method(const param_t &);                                   \
};                                                                      \
	\
	void BOOST_AUTO_TC_INVOKER( test_name )(const test_name::param_t &param) \
{                                                                       \
	test_name t;                                                        \
	t.test_method(param);                                               \
}                                                                       \
	\
	BOOST_AUTO_TU_REGISTRAR( test_name )(                                   \
	boost::unit_test::make_test_case(                                   \
	&BOOST_AUTO_TC_INVOKER( test_name ), #test_name,                 \
	(mbegin), (mend)));                                              \
	\
	void test_name::test_method(const param_t &param)                       \

// *******

#define BOOST_AUTO_PARAM_TEST_CASE( test_name, mbegin, mend )           \
	BOOST_FIXTURE_PARAM_TEST_CASE( test_name,                            \
	BOOST_AUTO_TEST_CASE_FIXTURE,         \
	mbegin, mend)