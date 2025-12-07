import React, { useState, useEffect, useRef } from 'react';
import { motion, AnimatePresence } from 'framer-motion';
import styles from './ChatbotWidget.module.css'; 
const ChatbotWidget = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [sessionId, setSessionId] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  useEffect(() => {
    // Basic session management
    let storedSessionId = localStorage.getItem('chatbotSessionId');
    if (!storedSessionId) {
      storedSessionId = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
      localStorage.setItem('chatbotSessionId', storedSessionId);
    }
    setSessionId(storedSessionId);
  }, []);

  const sendMessage = async (messageText, highlightedText = null) => {
    if (!messageText.trim() || isLoading) return;

    const userMessage = { id: messages.length + 1, sender: 'user', text: messageText };
    setMessages((prevMessages) => [...prevMessages, userMessage]);
    setInput('');
    setIsLoading(true);

    try {
      // Replace with your actual backend URL
      const backendUrl = process.env.CHATBOT_BACKEND_URL || 'http://localhost:8000';
      const response = await fetch(`${backendUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-API-Key': process.env.CHATBOT_API_KEY || 'your_fastapi_api_key_here',
        },
        body: JSON.stringify({
          query: messageText,
          session_id: sessionId,
          highlighted_text: highlightedText,
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const reader = response.body.getReader();
      const decoder = new TextDecoder('utf-8');
      let botResponseText = '';
      let isFirstChunk = true;

      while (true) {
        const { value, done } = await reader.read();
        if (done) break;

        const chunk = decoder.decode(value);
        // Assuming SSE format: data: {json_payload}

        const lines = chunk.split('\n\n').filter(line => line.startsWith('data: '));

        for (const line of lines) {
          const jsonString = line.substring(6); // Remove "data: "
          if (jsonString === '[DONE]') {
            setIsLoading(false);
            return;
          }
          const data = JSON.parse(jsonString);
          if (data.content) {
            botResponseText += data.content;
            if (isFirstChunk) {
              setMessages((prevMessages) => [
                ...prevMessages,
                { id: messages.length + 2, sender: 'bot', text: data.content },
              ]);
              isFirstChunk = false;
            } else {
              setMessages((prevMessages) =>
                prevMessages.map((msg, index) =>
                  index === prevMessages.length - 1 ? { ...msg, text: botResponseText } : msg
                )
              );
            }
          }
        }
      }
    } catch (error) {
      console.error('Chatbot message send error:', error);
      setMessages((prevMessages) => [
        ...prevMessages,
        { id: messages.length + 2, sender: 'bot', text: 'Oops! Something went wrong. Please try again.' },
      ]);
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter') {
      sendMessage(input);
    }
  };

  // Function to handle "select text + ask"
  const handleSelection = () => {
    const selection = window.getSelection();
    const selectedText = selection ? selection.toString().trim() : '';
    if (selectedText) {
      const question = prompt(`Ask a question about the selected text:\n"${selectedText}"`);
      if (question) {
        sendMessage(question, selectedText);
      }
    }
  };

  // Example: Listen for a hotkey to trigger selection mode (e.g., Ctrl+S)
  useEffect(() => {
    const handleKeyDown = (e) => {
      if (e.ctrlKey && e.key === 's') { // Ctrl+S
        e.preventDefault(); // Prevent browser save dialog
        handleSelection();
      }
    };
    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, []);


  return (
    <>
      <motion.button
        className={styles.toggleButton}
        onClick={() => setIsOpen(!isOpen)}
        whileHover={{ scale: 1.1 }}
        whileTap={{ scale: 0.9 }}
      >
        {isOpen ? 'X' : <img src="/img/ai-assistant-icon.svg" alt="AI Assistant" />} {/* AI Assistant Icon */}
      </motion.button>

      <AnimatePresence>
        {isOpen && (
          <motion.div
            className={styles.chatbotContainer}
            initial={{ opacity: 0, y: 100 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: 100 }}
            transition={{ type: 'spring', damping: 15, stiffness: 100 }}
          >
            <div className={styles.chatbotHeader}>
              <span>AI Assistant</span>
              <button onClick={() => setIsOpen(false)} className={styles.closeButton}>X</button>
            </div>
            <div className={styles.messagesContainer}>
              {messages.map((msg) => (
                <motion.div
                  key={msg.id}
                  className={clsx(styles.message, msg.sender === 'user' ? styles.userMessage : styles.botMessage)}
                  initial={{ opacity: 0, y: 20 }}
                  animate={{ opacity: 1, y: 0 }}
                  transition={{ duration: 0.3 }}
                >
                  {msg.text}
                </motion.div>
              ))}
              {isLoading && (
                <div className={styles.loadingMessage}>
                  <div className={styles.loadingDot}></div>
                  <div className={styles.loadingDot}></div>
                  <div className={styles.loadingDot}></div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
            <div className={styles.inputContainer}>
              <input
                type="text"
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask me anything about the book..."
                disabled={isLoading}
              />
              <button onClick={() => sendMessage(input)} disabled={isLoading}>Send</button>
            </div>
             <div className={styles.selectionHint}>
              Highlight text and press Ctrl+S to ask about it.
            </div>
          </motion.div>
        )}
      </AnimatePresence>
    </>
  );
};

export default ChatbotWidget;
