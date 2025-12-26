// Client module that renders the chatbot component
// Check if we're in the browser environment
if (typeof window !== 'undefined' && typeof document !== 'undefined') {
  // Create a container for the chatbot
  const chatbotContainer = document.createElement('div');
  chatbotContainer.id = 'chatbot-root';
  document.body.appendChild(chatbotContainer);

  // Render the chatbot component when React is available
  const renderChatbot = () => {
    if (typeof window !== 'undefined' && window.React && window.ReactDOM) {
      const React = window.React;
      const ReactDOMClient = window.ReactDOM;
      // Dynamically import the Chatbot component
      import('@site/src/components/Chatbot').then(ChatbotModule => {
        const Chatbot = ChatbotModule.default;
        if (Chatbot && ReactDOMClient.createRoot) {
          const root = ReactDOMClient.createRoot(chatbotContainer);
          root.render(React.createElement(Chatbot));
        }
      });
    }
  };

  // Check if React is already available, otherwise wait for it
  if (window.React && window.ReactDOM && window.ReactDOM.createRoot) {
    renderChatbot();
  } else {
    // Wait for React to load by checking periodically
    const checkReact = setInterval(() => {
      if (window.React && window.ReactDOM && window.ReactDOM.createRoot) {
        clearInterval(checkReact);
        renderChatbot();
      }
    }, 100);
  }
}