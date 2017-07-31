#include "Messenger.h"

/****************************************/
/****************************************/

swlexp::Messenger::Messenger()
    : m_transmitter(nullptr)
    , m_receiver(nullptr)
    , m_isFree(true)
{

}

/****************************************/
/****************************************/

swlexp::Messenger::~Messenger() {

}

/****************************************/
/****************************************/

void swlexp::Messenger::sendMsgTx(const argos::CByteArray& msgTx) {
    m_transmitter->SetData(msgTx);
    m_isFree = false;
}

/****************************************/
/****************************************/

void swlexp::Messenger::sendMsgTx(argos::CByteArray&& msgTx) {
    m_transmitter->SetData(msgTx);
    m_isFree = false;
}

/****************************************/
/****************************************/

const argos::CCI_RangeAndBearingSensor::TReadings& swlexp::Messenger::getMsgsRx() const {
    return m_receiver->GetReadings();
}

/****************************************/
/****************************************/

void swlexp::Messenger::registerCallback(MsgType type, Callback& callback) {
    m_callbacks.insert(std::make_pair(type, &callback));
}

/****************************************/
/****************************************/

void swlexp::Messenger::removeCallback(MsgType type, Callback& callback) {
    auto callbacksOfThatType = m_callbacks.equal_range(type);
    for (auto it = callbacksOfThatType.first; it != callbacksOfThatType.second; ++it) {
        if (it->second == &callback) {
            m_callbacks.erase(it);
            break;
        }
    }
}

/****************************************/
/****************************************/

void swlexp::Messenger::removeCallback(Callback& callback) {
    for (auto it = m_callbacks.begin(); it != m_callbacks.end(); ++it) {
        if (it->second == &callback) {
            m_callbacks.erase(it);
            break;
        }
    }
}

/****************************************/
/****************************************/

void swlexp::Messenger::controlStep() {

    const argos::CCI_RangeAndBearingSensor::TReadings& readings =
        m_receiver->GetReadings();

    for (const argos::CCI_RangeAndBearingSensor::SPacket& packet : readings) {
        MsgType type = (MsgType)packet.Data[0];
        auto callbacksOfThatType = m_callbacks.equal_range(type);
        for (auto it = callbacksOfThatType.first; it != callbacksOfThatType.second; ++it) {
            (*it->second)(packet);
        }
    }

    // Set outgoing message type to 0.
    if (!m_isFree) {
        m_isFree = true;
        m_transmitter->ClearData();
    }
    
}
