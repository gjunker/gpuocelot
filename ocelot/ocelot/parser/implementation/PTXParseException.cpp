#include "PTXParseException.h"

namespace parser {

const char* PTXParseException::what() const throw()
{
    return message.c_str();
}

PTXParseException::~PTXParseException() throw()
{
}

} // namespace parser
