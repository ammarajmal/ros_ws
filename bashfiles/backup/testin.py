"""
_summary_
"""
class TestClass:
    """_summary_
    """
    class_variable = "This is a class variable"

    def __init__(self, instance_variable):
        self.instance_variable = instance_variable
        print(self.instance_variable)

    def instance_method(self):
        """_summary_
        """
        print("This is an instance method")
        print("Instance variable value:", self.instance_variable)

    @classmethod
    def class_method(cls):
        '''_summary_'''
        print("This is a class method")
        print("Class variable value:", cls.class_variable)

    @staticmethod
    def static_method():
        """_summary_
        """
        print("This is a static method")

# Example usage
test_object = TestClass("This is an instance variable")
test_object.instance_method()
TestClass.class_method()
TestClass.static_method()
