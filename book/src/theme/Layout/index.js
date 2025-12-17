import React, { useState, useEffect, useCallback } from "react";
import Layout from "@theme-original/Layout";
import ChatbotWidget from "@site/src/components/ChatbotWidget";

const selectionMenuStyles = {
  position: "fixed",
  zIndex: 10001,
  padding: "4px",
  backgroundColor: "#333333",
  borderRadius: "8px",
  boxShadow: "0 4px 15px rgba(0,0,0,0.5)",
  transform: "translate(-50%, -100%)",
  transition: "opacity 0.2s, transform 0.2s",
  display: "flex",
  cursor: "pointer",
};

const buttonStyles = {
  backgroundColor: "#00A6FF",
  color: "#fff",
  border: "none",
  padding: "8px 12px",
  borderRadius: "6px",
  fontWeight: "bold",
  fontSize: "14px",
  cursor: "pointer",
  transition: "background-color 0.2s",
};

export default function LayoutWrapper(props) {
  const [selectedText, setSelectedText] = useState(null);
  const [selectionCoords, setSelectionCoords] = useState(null);
  const [initialQuery, setInitialQuery] = useState(null);

  const clearSelection = useCallback(() => {
    setSelectedText(null);
    setSelectionCoords(null);
    if (window.getSelection().rangeCount > 0) window.getSelection().removeAllRanges();
  }, []);

  const handleMouseUp = useCallback(() => {
    const selection = window.getSelection();
    const text = selection.toString().trim();
    if (!text || text.length < 4) return clearSelection();
    if (selection.anchorNode?.parentElement?.closest(".chatContainer")) return;
    if (selection.focusNode?.parentElement?.closest(".chatContainer")) return;

    const rect = selection.getRangeAt(0).getBoundingClientRect();
    setSelectedText(text);
    setSelectionCoords({ x: rect.left + rect.width / 2, y: rect.top - 10 });
  }, [clearSelection]);

  useEffect(() => {
    document.addEventListener("mouseup", handleMouseUp);
    return () => document.removeEventListener("mouseup", handleMouseUp);
  }, [handleMouseUp]);

  const handleAskAI = (e) => {
    e.stopPropagation();
    if (selectedText) {
      setInitialQuery(selectedText);
      clearSelection();
    }
  };

  return (
    <>
      <Layout {...props} />
      {selectedText && selectionCoords && (
        <div style={{ ...selectionMenuStyles, left: selectionCoords.x, top: selectionCoords.y }}>
          <button style={buttonStyles} onClick={handleAskAI}>
            💬 Ask AI about "{selectedText.substring(0, 15)}..."
          </button>
        </div>
      )}
      <ChatbotWidget
        selectedText={selectedText}
        clearSelectedText={clearSelection}
        initialQuery={initialQuery}
        clearInitialQuery={() => setInitialQuery(null)}
      />
    </>
  );
}
